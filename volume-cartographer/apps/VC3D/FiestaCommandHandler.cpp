#include "FiestaCommandHandler.hpp"

#include "FiestaDialogs.hpp"
#include "CommandLineToolRunner.hpp"
#include "SurfacePanelController.hpp"

#include "vc/core/fiesta/FiestaOps.hpp"
#include "vc/core/util/QuadSurface.hpp"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFutureWatcher>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QProgressDialog>
#include <QRegularExpression>
#include <QUuid>
#include <QtConcurrent/QtConcurrent>

#include <atomic>
#include <algorithm>
#include <array>
#include <filesystem>
#include <sstream>

namespace fs = std::filesystem;
using namespace vc::fiesta;

namespace
{

// First non-existing "<base>", "<base>_2", "<base>_3", ...
fs::path uniqueDir(const fs::path& base)
{
    if (!fs::exists(base))
        return base;
    for (int i = 2;; ++i) {
        fs::path candidate = base;
        candidate += "_" + std::to_string(i);
        if (!fs::exists(candidate))
            return candidate;
    }
}

QString writebackSummary(const vc::core::util::WriteBackStats& st)
{
    QString s = QObject::tr("grid %1x%2 at (%3,%4), %5 valid cells")
                    .arg(st.rect.width)
                    .arg(st.rect.height)
                    .arg(st.rect.x)
                    .arg(st.rect.y)
                    .arg(st.validCells);
    if (st.conflictCells || st.unplacedVertices || st.droppedTriangles)
        s += QObject::tr("  [conflicts %1, unplaced %2, dropped tris %3]")
                 .arg(st.conflictCells)
                 .arg(st.unplacedVertices)
                 .arg(st.droppedTriangles);
    return s;
}

// Sub-range progress: maps a per-ROI [0,1] fraction into slot i of n.
std::function<bool(float, const char*)> subProgress(
    const std::function<bool(float, const char*)>& outer, size_t i, size_t n)
{
    return [&outer, i, n](float f, const char* stage) {
        return outer((static_cast<float>(i) + f) / static_cast<float>(n), stage);
    };
}

// Mark a segment written by a ScrollFiesta operation: a panel-filterable tag
// plus a provenance block recording what produced it.
void tagFiestaSegment(
    QuadSurface& s, const char* tag, const QString& parentId, const char* op,
    int roi = -1, int piece = -1)
{
    if (!s.meta.is_object())
        s.meta = utils::Json::object();
    if (!s.meta.contains("tags") || !s.meta["tags"].is_object())
        s.meta["tags"] = utils::Json::object();
    s.meta["tags"][tag] = true;

    utils::Json f = utils::Json::object();
    f["op"] = op;
    f["parent"] = parentId.toStdString();
    if (roi >= 0)
        f["roi"] = roi;
    if (piece >= 0)
        f["piece"] = piece;
    s.meta["fiesta"] = f;
}

// Write `result` (a grid in the source frame at `rect`) into `target`'s
// grid. Valid AND invalid cells inside the intersection are copied —
// absence is deletion, so culled geometry disappears from the original too.
bool overwriteRegion(
    QuadSurface& target, QuadSurface& result, const cv::Rect& rect)
{
    cv::Mat_<cv::Vec3f>* P = target.rawPointsPtr();
    if (!P)
        return false;
    const cv::Mat_<cv::Vec3f> R = result.rawPoints();
    const cv::Rect inter = rect & cv::Rect(0, 0, P->cols, P->rows);
    for (int r = inter.y; r < inter.y + inter.height; ++r) {
        for (int c = inter.x; c < inter.x + inter.width; ++c) {
            (*P)(r, c) = R(r - rect.y, c - rect.x);
        }
    }
    return true;
}

}  // namespace

FiestaCommandHandler::FiestaCommandHandler(
    QWidget* parentWidget, SurfacePanelController* surfacePanel,
    SurfaceResolver resolver, HintContextResolver hintContextResolver,
    QObject* parent)
    : QObject(parent)
    , _parentWidget(parentWidget)
    , _surfacePanel(surfacePanel)
    , _resolver(std::move(resolver))
    , _hintContextResolver(std::move(hintContextResolver))
{
}

bool FiestaCommandHandler::ensureAvailable()
{
    auto& runtime = FiestaRuntime::instance();
    if (runtime.available())
        return true;
    QMessageBox::warning(
        _parentWidget, tr("ScrollFiesta unavailable"),
        tr("The ScrollFiesta library could not be loaded:\n\n%1")
            .arg(QString::fromStdString(runtime.unavailableReason())));
    return false;
}

void FiestaCommandHandler::runJob(
    const std::string& segmentId, const QString& progressTitle, WorkFn work)
{
    if (!ensureAvailable())
        return;
    if (_busy) {
        QMessageBox::warning(
            _parentWidget, tr("ScrollFiesta"),
            tr("A ScrollFiesta operation is already running."));
        return;
    }
    std::shared_ptr<QuadSurface> surf =
        _resolver ? _resolver(segmentId) : nullptr;
    if (!surf) {
        QMessageBox::warning(
            _parentWidget, tr("ScrollFiesta"),
            tr("Segment '%1' is not loaded.")
                .arg(QString::fromStdString(segmentId)));
        return;
    }

    _busy = true;
    emit statusMessage(progressTitle, 0);

    auto* progress = new QProgressDialog(
        progressTitle, tr("Cancel"), 0, 100, _parentWidget);
    progress->setWindowModality(Qt::NonModal);
    progress->setMinimumDuration(0);
    progress->setValue(0);
    progress->setAttribute(Qt::WA_DeleteOnClose);

    auto cancelled = std::make_shared<std::atomic_bool>(false);
    connect(progress, &QProgressDialog::canceled, this,
            [cancelled]() { cancelled->store(true); });

    // Worker-side progress: poll the cancel flag, marshal UI updates to the
    // GUI thread. Never touches Qt widgets from the worker directly.
    QPointer<QProgressDialog> progressGuard(progress);
    auto progressFn = [this, cancelled, progressGuard](
                          float fraction, const char* stage) -> bool {
        if (cancelled->load())
            return false;
        const int pct = static_cast<int>(fraction * 100.f);
        const QString stageText = stage ? QString::fromLatin1(stage) : QString();
        QMetaObject::invokeMethod(
            this,
            [progressGuard, pct, stageText]() {
                if (progressGuard) {
                    progressGuard->setValue(pct);
                    if (!stageText.isEmpty())
                        progressGuard->setLabelText(stageText);
                }
            },
            Qt::QueuedConnection);
        return true;
    };

    auto* watcher = new QFutureWatcher<JobResult>(this);
    connect(
        watcher, &QFutureWatcher<JobResult>::finished, this,
        [this, watcher, progressGuard]() {
            const JobResult result = watcher->result();
            watcher->deleteLater();
            _busy = false;
            if (progressGuard)
                progressGuard->close();

            if (result.cancelled) {
                emit statusMessage(
                    tr("%1: cancelled — nothing written").arg(result.title),
                    5000);
                return;
            }
            if (!result.ok) {
                emit statusMessage(tr("%1 failed").arg(result.title), 5000);
                QMessageBox::critical(
                    _parentWidget, result.title, result.error);
                return;
            }
            emit statusMessage(tr("%1 complete").arg(result.title), 5000);
            QMessageBox box(
                QMessageBox::Information, result.title, result.summary,
                QMessageBox::Ok, _parentWidget);
            if (!result.details.isEmpty())
                box.setDetailedText(result.details);
            box.exec();
            if (!result.written.isEmpty() && _surfacePanel) {
                QMetaObject::invokeMethod(
                    _surfacePanel.data(),
                    &SurfacePanelController::reloadSurfacesFromDisk,
                    Qt::QueuedConnection);
            }
        });

    watcher->setFuture(QtConcurrent::run(
        [surf, work = std::move(work), progressFn]() -> JobResult {
            return work(surf, progressFn);
        }));
}

void FiestaCommandHandler::onAudit(const std::string& segmentId)
{
    const QString id = QString::fromStdString(segmentId);
    runJob(
        segmentId, tr("ScrollFiesta: auditing %1...").arg(id),
        [id](std::shared_ptr<QuadSurface> surf,
             const std::function<bool(float, const char*)>& progress)
            -> JobResult {
            JobResult result;
            result.title = tr("Topology Audit — %1").arg(id);
            try {
                AuditReport report = auditQuadSurface(*surf, {}, progress);
                result.ok = true;
                result.summary =
                    tr("%1 components, %2 boundary loops, %3 non-manifold "
                       "edges, genus %4%5")
                        .arg(report.topo.n_components)
                        .arg(report.topo.n_boundary_loops)
                        .arg(report.topo.n_nonmanifold_edges)
                        .arg(report.topo.genus)
                        .arg(report.topo.is_disk
                                 ? tr(" — topological disk")
                                 : QString());
                result.details = QString::fromStdString(report.pretty());
            } catch (const FiestaCancelled&) {
                result.cancelled = true;
            } catch (const std::exception& e) {
                result.error = QString::fromUtf8(e.what());
            }
            return result;
        });
}

void FiestaCommandHandler::onClean(const std::string& segmentId)
{
    if (!ensureAvailable())
        return;
    FiestaCleanDialog dlg(
        _parentWidget, FiestaRuntime::instance().api(), /*allowInPlace=*/true);
    if (dlg.exec() != QDialog::Accepted)
        return;
    const sf_cleanup_config cfg = dlg.config();
    const bool overwrite = dlg.overwriteOriginal();

    const QString id = QString::fromStdString(segmentId);
    runJob(
        segmentId, tr("ScrollFiesta: cleaning %1...").arg(id),
        [id, cfg, overwrite](
            std::shared_ptr<QuadSurface> surf,
            const std::function<bool(float, const char*)>& progress)
            -> JobResult {
            JobResult result;
            result.title = tr("ScrollFiesta Clean — %1").arg(id);
            try {
                CleanupResult clean =
                    cleanupQuadSurfaceRoi(*surf, {}, &cfg, progress);

                if (overwrite) {
                    surf->saveSnapshot(-1, /*force=*/true);
                    if (!overwriteRegion(
                            *surf, *clean.surface, clean.writeback.rect))
                        throw std::runtime_error(
                            "could not write the result into the original grid");
                    surf->saveOverwrite();
                    result.written
                        << QString::fromStdString(surf->path.string());
                    result.summary =
                        tr("Overwrote %1 in place (disk backup saved — "
                           "right-click → Reload from backup to undo).\n"
                           "non-manifold edges %2 → %3, boundary loops %4 → %5")
                            .arg(id)
                            .arg(clean.before.topo.n_nonmanifold_edges)
                            .arg(clean.after.topo.n_nonmanifold_edges)
                            .arg(clean.before.topo.n_boundary_loops)
                            .arg(clean.after.topo.n_boundary_loops);
                } else {
                    const fs::path out = uniqueDir(
                        surf->path.parent_path() /
                        (id.toStdString() + "_fiesta_clean"));
                    tagFiestaSegment(
                        *clean.surface, "fiesta-clean", id, "clean");
                    clean.surface->save(
                        out.string(), out.filename().string());
                    result.written << QString::fromStdString(out.string());
                    result.summary =
                        tr("Saved as new segment %1\n"
                           "non-manifold edges %2 → %3, boundary loops %4 → %5")
                            .arg(QString::fromStdString(
                                out.filename().string()))
                            .arg(clean.before.topo.n_nonmanifold_edges)
                            .arg(clean.after.topo.n_nonmanifold_edges)
                            .arg(clean.before.topo.n_boundary_loops)
                            .arg(clean.after.topo.n_boundary_loops);
                }
                result.ok = true;
                result.details =
                    tr("Before:\n%1\nAfter:\n%2\nWrite-back: %3")
                        .arg(QString::fromStdString(clean.before.pretty()))
                        .arg(QString::fromStdString(clean.after.pretty()))
                        .arg(writebackSummary(clean.writeback));
            } catch (const FiestaCancelled&) {
                result.cancelled = true;
            } catch (const std::exception& e) {
                result.error = QString::fromUtf8(e.what());
            }
            return result;
        });
}

void FiestaCommandHandler::onDetangle(const std::string& segmentId)
{
    if (!ensureAvailable())
        return;
    FiestaDetangleDialog dlg(_parentWidget, FiestaRuntime::instance().api());
    if (dlg.exec() != QDialog::Accepted)
        return;
    const sf_detangle_config cfg = dlg.config();

    const QString id = QString::fromStdString(segmentId);
    runJob(
        segmentId, tr("ScrollFiesta: detangling %1...").arg(id),
        [id, cfg](std::shared_ptr<QuadSurface> surf,
                  const std::function<bool(float, const char*)>& progress)
            -> JobResult {
            JobResult result;
            result.title = tr("ScrollFiesta Detangle — %1").arg(id);
            try {
                DetangleResult det =
                    detangleQuadSurface(*surf, {}, &cfg, progress);

                std::ostringstream details;
                int k = 0;
                for (auto& piece : det.pieces) {
                    const fs::path out = uniqueDir(
                        surf->path.parent_path() /
                        (id.toStdString() + "_fiesta_s" + std::to_string(k)));
                    tagFiestaSegment(
                        *piece.surface, "fiesta-split", id, "detangle", -1, k);
                    piece.surface->save(
                        out.string(), out.filename().string());
                    result.written
                        << QString::fromStdString(out.string());
                    details << out.filename().string() << ": "
                            << writebackSummary(piece.writeback)
                                   .toStdString()
                            << "\n";
                    ++k;
                }
                result.ok = true;
                if (det.pieces.size() == 1) {
                    result.summary = tr(
                        "No split was needed (the surface self-gated to one "
                        "sheet).\nWrote 1 segment.");
                } else {
                    result.summary =
                        tr("Split into %1 pieces (peel %2, developability %3, "
                           "bridge %4, overlap %5).\nOriginal untouched.")
                            .arg(det.pieces.size())
                            .arg(det.report.peel_splits)
                            .arg(det.report.dev_splits)
                            .arg(det.report.bridge_splits)
                            .arg(det.report.overlap_splits);
                }
                result.details = QString::fromStdString(details.str());
            } catch (const FiestaCancelled&) {
                result.cancelled = true;
            } catch (const std::exception& e) {
                result.error = QString::fromUtf8(e.what());
            }
            return result;
        });
}

void FiestaCommandHandler::onGenerateSpiralHints(
    const std::string& segmentId, std::vector<cv::Rect> rois)
{
    if (!ensureAvailable())
        return;
    if (_busy) {
        QMessageBox::warning(
            _parentWidget, tr("ScrollFiesta"),
            tr("A ScrollFiesta operation is already running."));
        return;
    }
    if (!_cmdRunner) {
        QMessageBox::critical(
            _parentWidget, tr("ScrollFiesta"),
            tr("The external-tool runner is not available."));
        return;
    }
    if (rois.empty()) {
        QMessageBox::warning(
            _parentWidget, tr("ScrollFiesta"),
            tr("Draw at least one rectangular selection first."));
        return;
    }

    std::shared_ptr<QuadSurface> surf =
        _resolver ? _resolver(segmentId) : nullptr;
    if (!surf || surf->path.empty()) {
        QMessageBox::warning(
            _parentWidget, tr("ScrollFiesta"),
            tr("Segment '%1' does not resolve to an on-disk TIFXYZ directory.")
                .arg(QString::fromStdString(segmentId)));
        return;
    }

    const auto bbox = surf->bbox();
    const std::array<double, 3> suggestedAxisPointZyx{
        0.5 * (static_cast<double>(bbox.low[2]) + bbox.high[2]),
        0.5 * (static_cast<double>(bbox.low[1]) + bbox.high[1]),
        0.5 * (static_cast<double>(bbox.low[0]) + bbox.high[0])};
    FiestaCleanDialog dlg(
        _parentWidget, FiestaRuntime::instance().api(), /*allowInPlace=*/false,
        suggestedAxisPointZyx);
    if (dlg.exec() != QDialog::Accepted)
        return;
    const sf_cleanup_config cfg = dlg.config();

    QString sourcePath = QString::fromStdString(surf->path.string());
    QString outputRoot =
        QString::fromStdString(surf->path.parent_path().string());
    QString sourceId = QString::fromStdString(segmentId);
    QString safeId = sourceId;
    safeId.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9_.-]+")),
                   QStringLiteral("_"));
    const QString runId =
        safeId + QStringLiteral("_scroll_hint_") +
        QDateTime::currentDateTimeUtc().toString(
            QStringLiteral("yyyyMMdd'T'HHmmss'Z'"));
    _externalManifestPath = QDir(outputRoot).filePath(
        runId + QStringLiteral(".scrollfiesta-hints.json"));
    const QString cancelPath = QDir(QDir::tempPath()).filePath(
        QStringLiteral("vc3d-scrollfiesta-%1.cancel")
            .arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));

    QString executable = QDir(QCoreApplication::applicationDirPath()).filePath(
#ifdef Q_OS_WIN
        QStringLiteral("vc_fiesta.exe")
#else
        QStringLiteral("vc_fiesta")
#endif
    );
    QStringList args{
        QStringLiteral("hints"),
        sourcePath,
        outputRoot,
        QStringLiteral("--id-prefix=%1").arg(runId),
        QStringLiteral("--json=%1").arg(_externalManifestPath),
        QStringLiteral("--cancel-file=%1").arg(cancelPath),
        QStringLiteral("--axis-point-zyx=%1").arg(dlg.axisPointZyx()),
        QStringLiteral("--axis-direction-zyx=%1").arg(dlg.axisDirectionZyx()),
        QStringLiteral("--wrap-spacing=%1")
            .arg(dlg.wrapSpacing(), 0, 'g', 17),
        QStringLiteral("--manifold=%1").arg(cfg.manifold_repair ? 1 : 0),
        QStringLiteral("--pinholes=%1").arg(cfg.fill_pinholes ? 1 : 0),
        QStringLiteral("--sliver=%1").arg(cfg.sliver_cleanup ? 1 : 0),
        QStringLiteral("--cull-frac=%1")
            .arg(cfg.cull_min_area_frac, 0, 'g', 9),
        QStringLiteral("--progress")};
    for (const auto& roi : rois) {
        args << QStringLiteral("--roi=%1,%2,%3,%4")
                    .arg(roi.x)
                    .arg(roi.y)
                    .arg(roi.width)
                    .arg(roi.height);
    }

    const HintContext context =
        _hintContextResolver ? _hintContextResolver() : HintContext{};
    if (!context.volumeId.isEmpty())
        args << QStringLiteral("--volume-id=%1").arg(context.volumeId);
    if (!context.volumeLocation.isEmpty())
        args << QStringLiteral("--volume-location=%1")
                    .arg(context.volumeLocation);
    if (context.voxelSize > 0.0)
        args << QStringLiteral("--voxel-size=%1")
                    .arg(context.voxelSize, 0, 'g', 17);
    for (const auto& tag : context.volumeTags)
        args << QStringLiteral("--volume-tag=%1").arg(tag);

    _busy = true;
    _externalCancelRequested = false;
    _externalAddToCurrentFit = dlg.addToCurrentFit();
    _externalLineBuffer.clear();
    _externalOutputTail.clear();
    emit statusMessage(
        tr("ScrollFiesta: generating %1 spiral hint(s) from %2...")
            .arg(rois.size())
            .arg(sourceId),
        0);

    auto* progress = new QProgressDialog(
        tr("Generating %1 unverified spiral hint(s)...").arg(rois.size()),
        tr("Cancel"), 0, 100, _parentWidget);
    progress->setWindowTitle(tr("ScrollFiesta Spiral Hints"));
    progress->setWindowModality(Qt::NonModal);
    progress->setMinimumDuration(0);
    progress->setAutoClose(false);
    progress->setValue(0);
    progress->setAttribute(Qt::WA_DeleteOnClose);
    _externalProgress = progress;
    connect(progress, &QProgressDialog::canceled, this, [this]() {
        if (_externalCancelRequested)
            return;
        _externalCancelRequested = true;
        if (_externalProgress) {
            _externalProgress->setLabelText(
                tr("Cancelling safely; cleaning staged outputs..."));
        }
        if (_cmdRunner)
            _cmdRunner->cancel();
    });

    _externalOutputConnection = connect(
        _cmdRunner, &CommandLineToolRunner::consoleOutputReceived, this,
        [this](const QString& chunk) {
            _externalOutputTail += chunk;
            if (_externalOutputTail.size() > 16000)
                _externalOutputTail = _externalOutputTail.right(16000);
            _externalLineBuffer += chunk;
            int newline = -1;
            static const QRegularExpression progressLine(
                QStringLiteral("^PROGRESS\\s+(\\d+)\\s*(.*)$"));
            while ((newline = _externalLineBuffer.indexOf('\n')) >= 0) {
                QString line = _externalLineBuffer.left(newline).trimmed();
                _externalLineBuffer.remove(0, newline + 1);
                const auto match = progressLine.match(line);
                if (match.hasMatch() && _externalProgress) {
                    _externalProgress->setValue(
                        std::clamp(match.captured(1).toInt(), 0, 100));
                    const QString stage = match.captured(2).trimmed();
                    if (!stage.isEmpty())
                        _externalProgress->setLabelText(stage);
                }
            }
        });

    _externalFinishedConnection = connect(
        _cmdRunner, &CommandLineToolRunner::toolFinished, this,
        [this, sourceId](CommandLineToolRunner::Tool tool, bool success,
                         const QString& message, const QString&, bool) {
            if (tool != CommandLineToolRunner::Tool::CustomCommand)
                return;
            disconnect(_externalOutputConnection);
            disconnect(_externalFinishedConnection);
            if (_externalProgress) {
                if (success)
                    _externalProgress->setValue(100);
                _externalProgress->close();
            }
            _busy = false;

            if (_externalCancelRequested && !success) {
                emit statusMessage(
                    tr("ScrollFiesta spiral hints cancelled — nothing written"),
                    5000);
                return;
            }
            if (!success) {
                emit statusMessage(tr("ScrollFiesta spiral hints failed"), 5000);
                QMessageBox box(
                    QMessageBox::Critical, tr("ScrollFiesta Spiral Hints"),
                    message, QMessageBox::Ok, _parentWidget);
                if (!_externalOutputTail.trimmed().isEmpty())
                    box.setDetailedText(_externalOutputTail.trimmed());
                box.exec();
                return;
            }

            QFile manifestFile(_externalManifestPath);
            if (!manifestFile.open(QIODevice::ReadOnly)) {
                QMessageBox::critical(
                    _parentWidget, tr("ScrollFiesta Spiral Hints"),
                    tr("The job succeeded but its manifest could not be read:\n%1")
                        .arg(_externalManifestPath));
                return;
            }
            QJsonParseError parseError;
            const QJsonDocument manifest = QJsonDocument::fromJson(
                manifestFile.readAll(), &parseError);
            if (parseError.error != QJsonParseError::NoError ||
                !manifest.isObject() ||
                manifest.object().value(QStringLiteral("schema")).toString() !=
                    QStringLiteral("vc3d-scrollfiesta-hints-v1")) {
                QMessageBox::critical(
                    _parentWidget, tr("ScrollFiesta Spiral Hints"),
                    tr("The job produced an invalid hint manifest:\n%1\n\n%2")
                        .arg(_externalManifestPath, parseError.errorString()));
                return;
            }

            QStringList ids;
            QStringList paths;
            QStringList details;
            const QJsonArray outputs = manifest.object()
                                           .value(QStringLiteral("outputs"))
                                           .toArray();
            for (const auto& value : outputs) {
                const QJsonObject entry = value.toObject();
                const QString id = entry.value(QStringLiteral("id")).toString();
                const QString path =
                    entry.value(QStringLiteral("path")).toString();
                if (id.isEmpty() || path.isEmpty())
                    continue;
                ids << id;
                paths << path;
                const auto writeback =
                    entry.value(QStringLiteral("writeback")).toObject();
                details << tr("%1 — %2 valid cells\n%3")
                               .arg(id)
                               .arg(writeback.value(
                                        QStringLiteral("valid_cells"))
                                        .toVariant()
                                        .toULongLong())
                               .arg(path);
            }
            if (ids.isEmpty()) {
                QMessageBox::critical(
                    _parentWidget, tr("ScrollFiesta Spiral Hints"),
                    tr("The manifest contains no importable outputs."));
                return;
            }

            if (_surfacePanel) {
                _surfacePanel->reloadSurfacesFromDisk();
                QString activationError;
                _surfacePanel->activateSurfaceById(
                    ids.front().toStdString(), &activationError);
            }
            emit spiralHintsGenerated(ids, paths, _externalAddToCurrentFit);
            emit statusMessage(
                tr("Generated and imported %1 unverified spiral hint(s)")
                    .arg(ids.size()),
                5000);

            QMessageBox box(
                QMessageBox::Information, tr("ScrollFiesta Spiral Hints"),
                tr("Generated %1 unverified spiral hint(s) from %2.\n"
                   "Source untouched; first hint activated in VC3D.%3")
                    .arg(ids.size())
                    .arg(sourceId)
                    .arg(_externalAddToCurrentFit
                             ? tr("\nHints were offered to the current Spiral fit.")
                             : QString()),
                QMessageBox::Ok, _parentWidget);
            box.setDetailedText(
                details.join(QStringLiteral("\n\n")) +
                tr("\n\nManifest:\n%1").arg(_externalManifestPath));
            box.exec();
        });

    _cmdRunner->setNextCancellationFile(cancelPath);
    if (!_cmdRunner->executeCustomCommand(
            executable, args, tr("ScrollFiesta spiral hints"),
            CommandLineToolRunner::ExecutionOptions::silent())) {
        disconnect(_externalOutputConnection);
        disconnect(_externalFinishedConnection);
        _busy = false;
        if (_externalProgress)
            _externalProgress->close();
        QMessageBox::critical(
            _parentWidget, tr("ScrollFiesta Spiral Hints"),
            tr("Could not start the external ScrollFiesta job."));
    }
}

void FiestaCommandHandler::onDetangleRois(
    const std::string& segmentId, std::vector<cv::Rect> rois)
{
    if (!ensureAvailable())
        return;
    FiestaDetangleDialog dlg(_parentWidget, FiestaRuntime::instance().api());
    if (dlg.exec() != QDialog::Accepted)
        return;
    const sf_detangle_config cfg = dlg.config();

    const QString id = QString::fromStdString(segmentId);
    runJob(
        segmentId,
        tr("ScrollFiesta: detangling %1 selection(s) of %2...")
            .arg(rois.size())
            .arg(id),
        [id, cfg, rois = std::move(rois)](
            std::shared_ptr<QuadSurface> surf,
            const std::function<bool(float, const char*)>& progress)
            -> JobResult {
            JobResult result;
            result.title = tr("ScrollFiesta Detangle Selection — %1").arg(id);
            std::ostringstream details;
            size_t total_pieces = 0;
            try {
                for (size_t k = 0; k < rois.size(); ++k) {
                    DetangleResult det = detangleQuadSurface(
                        *surf, rois[k], &cfg,
                        subProgress(progress, k, rois.size()));
                    int piece = 0;
                    for (auto& p : det.pieces) {
                        const fs::path out = uniqueDir(
                            surf->path.parent_path() /
                            (id.toStdString() + "_fiesta_roi" +
                             std::to_string(k) + "_s" +
                             std::to_string(piece)));
                        tagFiestaSegment(
                            *p.surface, "fiesta-split", id, "detangle",
                            static_cast<int>(k), piece);
                        p.surface->save(out.string(),
                                        out.filename().string());
                        result.written
                            << QString::fromStdString(out.string());
                        details << out.filename().string() << ": "
                                << writebackSummary(p.writeback)
                                       .toStdString()
                                << "\n";
                        ++piece;
                    }
                    total_pieces += det.pieces.size();
                }
                result.ok = true;
                result.summary =
                    tr("%1 selection(s) -> %2 piece(s), saved as new "
                       "segments.\nOriginal untouched.")
                        .arg(rois.size())
                        .arg(total_pieces);
                result.details = QString::fromStdString(details.str());
            } catch (const FiestaCancelled&) {
                result.cancelled = true;
            } catch (const std::exception& e) {
                result.error = QString::fromUtf8(e.what());
            }
            return result;
        });
}
