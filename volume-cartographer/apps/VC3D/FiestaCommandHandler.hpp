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

#include <functional>
#include <memory>
#include <string>

class QuadSurface;
class QWidget;
class SurfacePanelController;

class FiestaCommandHandler : public QObject
{
    Q_OBJECT

public:
    using SurfaceResolver =
        std::function<std::shared_ptr<QuadSurface>(const std::string&)>;

    FiestaCommandHandler(
        QWidget* parentWidget, SurfacePanelController* surfacePanel,
        SurfaceResolver resolver, QObject* parent = nullptr);

public slots:
    void onAudit(const std::string& segmentId);
    void onClean(const std::string& segmentId);
    void onDetangle(const std::string& segmentId);

signals:
    void statusMessage(const QString& text, int timeoutMs);

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
    bool _busy = false;
};
