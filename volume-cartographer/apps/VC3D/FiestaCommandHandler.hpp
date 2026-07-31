#pragma once

// FiestaCommandHandler — GUI entry points for the ScrollFiesta operations
// (topology audit / cleanup / detangle) on a segment. Work runs in-process on
// a QtConcurrent worker with real progress + cancellation through the
// ScrollFiesta C API; results are written as NEW segments next to the source
// (the original is never touched), then the surface panel reloads.
//
// This file is only compiled when the build fetches ScrollFiesta
// (VC_WITH_SCROLLFIESTA); call sites in shared files guard with
// #ifdef VC_HAVE_SCROLLFIESTA.

#include <QObject>
#include <QPointer>
#include <QString>
#include <QStringList>

#include <opencv2/core.hpp>

#include <functional>
#include <memory>
#include <string>

class QuadSurface;
class QWidget;
class SurfacePanelController;
class CommandLineToolRunner;
class QProgressDialog;

class FiestaCommandHandler : public QObject
{
    Q_OBJECT

public:
    struct HintContext {
        QString volumeId;
        QString volumeLocation;
        double voxelSize = 0.0;
        QStringList volumeTags;
    };

    using SurfaceResolver =
        std::function<std::shared_ptr<QuadSurface>(const std::string&)>;
    using HintContextResolver = std::function<HintContext()>;

    FiestaCommandHandler(
        QWidget* parentWidget, SurfacePanelController* surfacePanel,
        SurfaceResolver resolver, HintContextResolver hintContextResolver,
        QObject* parent = nullptr);

    void setCommandLineToolRunner(CommandLineToolRunner* runner)
    {
        _cmdRunner = runner;
    }

public slots:
    void onAudit(const std::string& segmentId);
    void onClean(const std::string& segmentId);
    void onDetangle(const std::string& segmentId);
    // Generate low-trust scroll-diffeomorphism hints from the viewer's BBox
    // selections. Each ROI is always saved as a new, unverified segment; the
    // source segment is never modified.
    void onGenerateSpiralHints(
        const std::string& segmentId, std::vector<cv::Rect> rois);
    void onDetangleRois(const std::string& segmentId, std::vector<cv::Rect> rois);

signals:
    void statusMessage(const QString& text, int timeoutMs);
    void spiralHintsGenerated(
        const QStringList& ids, const QStringList& paths,
        bool addToCurrentFit);

private:
    struct JobResult {
        bool ok = false;
        bool cancelled = false;
        QString error;
        QString title;
        QString summary;   // headline for the message box
        QString details;   // long text for setDetailedText
        QStringList written;
    };

    using WorkFn = std::function<JobResult(
        std::shared_ptr<QuadSurface>,
        const std::function<bool(float, const char*)>&)>;

    // Availability gate: pops a message box with the loader's reason when the
    // scrollfiesta library is absent. Returns false in that case.
    bool ensureAvailable();
    void runJob(
        const std::string& segmentId, const QString& progressTitle,
        WorkFn work);

    QWidget* _parentWidget = nullptr;
    QPointer<SurfacePanelController> _surfacePanel;
    SurfaceResolver _resolver;
    HintContextResolver _hintContextResolver;
    QPointer<CommandLineToolRunner> _cmdRunner;
    QPointer<QProgressDialog> _externalProgress;
    QMetaObject::Connection _externalOutputConnection;
    QMetaObject::Connection _externalFinishedConnection;
    QString _externalLineBuffer;
    QString _externalOutputTail;
    QString _externalManifestPath;
    bool _externalCancelRequested = false;
    bool _externalAddToCurrentFit = false;
    bool _busy = false;
};
