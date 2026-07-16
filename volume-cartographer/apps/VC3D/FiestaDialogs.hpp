#pragma once

// Parameter dialogs for the ScrollFiesta operations (ToolDialogs style:
// session-static defaults so repeated runs remember the last choice, code
// defaults sourced from the loaded library's *_config_default()).
// Compiled only in VC_WITH_SCROLLFIESTA builds.

#include <scrollfiesta.h>

#include <QDialog>

class QCheckBox;
class QDoubleSpinBox;
class QRadioButton;
class QSpinBox;

class FiestaCleanDialog : public QDialog
{
    Q_OBJECT

public:
    // `api` supplies the library defaults; `allowInPlace` shows the
    // overwrite-original result mode (disk backup + saveOverwrite).
    FiestaCleanDialog(QWidget* parent, const sf_api* api, bool allowInPlace);

    sf_cleanup_config config() const;
    bool overwriteOriginal() const;

    void accept() override;

private:
    sf_cleanup_config _defaults;

    QCheckBox* _cbManifold{nullptr};
    QCheckBox* _cbPinholes{nullptr};
    QCheckBox* _cbSliver{nullptr};
    QDoubleSpinBox* _spCull{nullptr};
    QRadioButton* _rbNewSegment{nullptr};
    QRadioButton* _rbOverwrite{nullptr};

    static bool s_haveSession;
    static bool s_manifold;
    static bool s_pinholes;
    static bool s_sliver;
    static double s_cullFrac;
    static bool s_overwrite;
};

class FiestaDetangleDialog : public QDialog
{
    Q_OBJECT

public:
    FiestaDetangleDialog(QWidget* parent, const sf_api* api);

    sf_detangle_config config() const;

    void accept() override;

private:
    sf_detangle_config _defaults;

    QCheckBox* _cbPeel{nullptr};
    QCheckBox* _cbDev{nullptr};
    QCheckBox* _cbBridge{nullptr};
    QCheckBox* _cbOverlap{nullptr};
    QSpinBox* _spMinComp{nullptr};
    QDoubleSpinBox* _spTimeout{nullptr};

    static bool s_haveSession;
    static bool s_peel;
    static bool s_dev;
    static bool s_bridge;
    static bool s_overlap;
    static int s_minComp;
    static double s_timeout;
};
