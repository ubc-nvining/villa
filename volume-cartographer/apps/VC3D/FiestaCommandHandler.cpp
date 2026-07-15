#include "FiestaCommandHandler.hpp"

#include "SurfacePanelController.hpp"

#include "vc/core/fiesta/FiestaOps.hpp"
#include "vc/core/util/QuadSurface.hpp"

#include <QFutureWatcher>
#include <QMessageBox>
#include <QProgressDialog>
#include <QtConcurrent/QtConcurrent>

#include <atomic>
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

}  // namespace

FiestaCommandHandler::FiestaCommandHandler(
    QWidget* parentWidget, SurfacePanelController* surfacePanel,
    SurfaceResolver resolver, QObject* parent)
    : QObject(parent)
    , _parentWidget(parentWidget)
    , _surfacePanel(surfacePanel)
    , _resolver(std::move(resolver))
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

namespace
{

// Sub-range progress: maps a per-ROI [0,1] fraction into slot i of n.
std::function<bool(float, const char*)> subProgress(
    const std::function<bool(float, const char*)>& outer, size_t i, size_t n)
{
    return [&outer, i, n](float f, const char* stage) {
        return outer((static_cast<float>(i) + f) / static_cast<float>(n), stage);
    };
}

}  // namespace

void FiestaCommandHandler::onCleanRois(
    const std::string& segmentId, std::vector<cv::Rect> rois)
{
    const QString id = QString::fromStdString(segmentId);
    runJob(
        segmentId,
        tr("ScrollFiesta: cleaning %1 selection(s) of %2...")
            .arg(rois.size())
            .arg(id),
        [id, rois = std::move(rois)](
            std::shared_ptr<QuadSurface> surf,
            const std::function<bool(float, const char*)>& progress)
            -> JobResult {
            JobResult result;
            result.title = tr("ScrollFiesta Clean Selection — %1").arg(id);
            std::ostringstream details;
            try {
                for (size_t k = 0; k < rois.size(); ++k) {
                    CleanupResult clean = cleanupQuadSurfaceRoi(
                        *surf, rois[k], nullptr,
                        subProgress(progress, k, rois.size()));
                    const fs::path out = uniqueDir(
                        surf->path.parent_path() /
                        (id.toStdString() + "_fiesta_roi" + std::to_string(k) +
                         "_clean"));
                    clean.surface->save(out.string(), out.filename().string());
                    result.written << QString::fromStdString(out.string());
                    details << out.filename().string() << ": non-manifold "
                            << clean.before.topo.n_nonmanifold_edges << " -> "
                            << clean.after.topo.n_nonmanifold_edges << ", "
                            << writebackSummary(clean.writeback).toStdString()
                            << "\n";
                }
                result.ok = true;
                result.summary =
                    tr("Cleaned %1 selection(s); results saved as new "
                       "segments.\nOriginal untouched.")
                        .arg(rois.size());
                result.details = QString::fromStdString(details.str());
            } catch (const FiestaCancelled&) {
                result.cancelled = true;
            } catch (const std::exception& e) {
                result.error = QString::fromUtf8(e.what());
            }
            return result;
        });
}

void FiestaCommandHandler::onDetangleRois(
    const std::string& segmentId, std::vector<cv::Rect> rois)
{
    const QString id = QString::fromStdString(segmentId);
    runJob(
        segmentId,
        tr("ScrollFiesta: detangling %1 selection(s) of %2...")
            .arg(rois.size())
            .arg(id),
        [id, rois = std::move(rois)](
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
                        *surf, rois[k], nullptr,
                        subProgress(progress, k, rois.size()));
                    int piece = 0;
                    for (auto& p : det.pieces) {
                        const fs::path out = uniqueDir(
                            surf->path.parent_path() /
                            (id.toStdString() + "_fiesta_roi" +
                             std::to_string(k) + "_s" +
                             std::to_string(piece++)));
                        p.surface->save(out.string(),
                                        out.filename().string());
                        result.written
                            << QString::fromStdString(out.string());
                        details << out.filename().string() << ": "
                                << writebackSummary(p.writeback)
                                       .toStdString()
                                << "\n";
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
    const QString id = QString::fromStdString(segmentId);
    runJob(
        segmentId, tr("ScrollFiesta: cleaning %1...").arg(id),
        [id](std::shared_ptr<QuadSurface> surf,
             const std::function<bool(float, const char*)>& progress)
            -> JobResult {
            JobResult result;
            result.title = tr("ScrollFiesta Clean — %1").arg(id);
            try {
                CleanupResult clean =
                    cleanupQuadSurfaceRoi(*surf, {}, nullptr, progress);

                const fs::path out = uniqueDir(
                    surf->path.parent_path() /
                    (id.toStdString() + "_fiesta_clean"));
                clean.surface->save(
                    out.string(), out.filename().string());

                result.ok = true;
                result.written << QString::fromStdString(out.string());
                result.summary =
                    tr("Saved as new segment %1\n"
                       "non-manifold edges %2 → %3, boundary loops %4 → %5")
                        .arg(QString::fromStdString(out.filename().string()))
                        .arg(clean.before.topo.n_nonmanifold_edges)
                        .arg(clean.after.topo.n_nonmanifold_edges)
                        .arg(clean.before.topo.n_boundary_loops)
                        .arg(clean.after.topo.n_boundary_loops);
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
    const QString id = QString::fromStdString(segmentId);
    runJob(
        segmentId, tr("ScrollFiesta: detangling %1...").arg(id),
        [id](std::shared_ptr<QuadSurface> surf,
             const std::function<bool(float, const char*)>& progress)
            -> JobResult {
            JobResult result;
            result.title = tr("ScrollFiesta Detangle — %1").arg(id);
            try {
                DetangleResult det =
                    detangleQuadSurface(*surf, {}, nullptr, progress);

                std::ostringstream details;
                int k = 0;
                for (auto& piece : det.pieces) {
                    const fs::path out = uniqueDir(
                        surf->path.parent_path() /
                        (id.toStdString() + "_fiesta_s" + std::to_string(k)));
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
