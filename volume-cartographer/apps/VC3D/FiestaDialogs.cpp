#include "FiestaDialogs.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QRadioButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include <cmath>

// ─────────────────────────────── clean ─────────────────────────────────────

bool FiestaCleanDialog::s_haveSession = false;
bool FiestaCleanDialog::s_manifold = true;
bool FiestaCleanDialog::s_pinholes = true;
bool FiestaCleanDialog::s_sliver = false;
double FiestaCleanDialog::s_cullFrac = 0.0;
bool FiestaCleanDialog::s_overwrite = false;
bool FiestaCleanDialog::s_haveHintGeometry = false;
std::array<double, 3> FiestaCleanDialog::s_axisPoint{0.0, 0.0, 0.0};
std::array<double, 3> FiestaCleanDialog::s_axisDirection{1.0, 0.0, 0.0};
double FiestaCleanDialog::s_wrapSpacing = 9.5;
bool FiestaCleanDialog::s_addToFit = true;

FiestaCleanDialog::FiestaCleanDialog(
    QWidget* parent, const sf_api* api, bool allowInPlace,
    std::array<double, 3> suggestedAxisPointZyx)
    : QDialog(parent)
{
    setWindowTitle(allowInPlace ? tr("ScrollFiesta Clean")
                                : tr("ScrollFiesta Spiral Hints"));
    _defaults = api->cleanup_config_default();

    const bool manifold =
        s_haveSession ? s_manifold : _defaults.manifold_repair != 0;
    const bool pinholes =
        s_haveSession ? s_pinholes : _defaults.fill_pinholes != 0;
    const bool sliver =
        s_haveSession ? s_sliver : _defaults.sliver_cleanup != 0;
    const double cull =
        s_haveSession ? s_cullFrac : static_cast<double>(_defaults.cull_min_area_frac);

    auto* layout = new QVBoxLayout(this);

    if (!allowInPlace) {
        auto* hint = new QLabel(
            tr("Generate new, unverified surface patches for the scroll "
               "diffeomorphism. The source segment is never modified."),
            this);
        hint->setWordWrap(true);
        layout->addWidget(hint);

        auto* geometry = new QGroupBox(
            tr("Scroll geometry (volume coordinates)"), this);
        auto* geometryForm = new QFormLayout(geometry);
        auto makeTriple = [geometry](
                              std::array<QDoubleSpinBox*, 3>& fields,
                              const std::array<double, 3>& values,
                              double minimum, double maximum, int decimals) {
            auto* row = new QWidget(geometry);
            auto* rowLayout = new QHBoxLayout(row);
            rowLayout->setContentsMargins(0, 0, 0, 0);
            const char* names[] = {"Z", "Y", "X"};
            for (size_t i = 0; i < fields.size(); ++i) {
                rowLayout->addWidget(new QLabel(QString::fromLatin1(names[i]), row));
                fields[i] = new QDoubleSpinBox(row);
                fields[i]->setRange(minimum, maximum);
                fields[i]->setDecimals(decimals);
                fields[i]->setValue(values[i]);
                rowLayout->addWidget(fields[i]);
            }
            return row;
        };
        const auto axisPoint =
            s_haveHintGeometry ? s_axisPoint : suggestedAxisPointZyx;
        geometryForm->addRow(
            tr("Axis point (Z,Y,X):"),
            makeTriple(_spAxisPoint, axisPoint, -1000000.0, 1000000.0, 3));
        geometryForm->addRow(
            tr("Axis direction (Z,Y,X):"),
            makeTriple(
                _spAxisDirection, s_axisDirection, -1.0, 1.0, 6));
        _spWrapSpacing = new QDoubleSpinBox(geometry);
        _spWrapSpacing->setRange(0.001, 100000.0);
        _spWrapSpacing->setDecimals(4);
        _spWrapSpacing->setValue(s_wrapSpacing);
        _spWrapSpacing->setToolTip(
            tr("Expected center-to-center spacing between adjacent wraps, in voxels."));
        geometryForm->addRow(tr("Wrap spacing (voxels):"), _spWrapSpacing);
        layout->addWidget(geometry);
    }

    auto* stages = new QGroupBox(tr("Stages"), this);
    auto* form = new QFormLayout(stages);
    _cbManifold = new QCheckBox(tr("Manifold repair (resolve >2-face edges, split pinch vertices, re-wind)"), stages);
    _cbManifold->setChecked(manifold);
    form->addRow(_cbManifold);
    _cbPinholes = new QCheckBox(tr("Fill pinholes (exact single-triangle holes)"), stages);
    _cbPinholes->setChecked(pinholes);
    form->addRow(_cbPinholes);
    _cbSliver = new QCheckBox(tr("Sliver cleanup (edge flips + guarded collapses)"), stages);
    _cbSliver->setChecked(sliver);
    form->addRow(_cbSliver);
    _spCull = new QDoubleSpinBox(stages);
    _spCull->setRange(0.0, 0.5);
    _spCull->setDecimals(3);
    _spCull->setSingleStep(0.005);
    _spCull->setValue(cull);
    _spCull->setToolTip(tr("Drop connectivity components whose surface area is below "
                           "this fraction of the total. 0 = keep everything "
                           "(the batch pipeline uses 0.02)."));
    form->addRow(tr("Cull components below area fraction:"), _spCull);
    layout->addWidget(stages);

    auto* resultBox = new QGroupBox(tr("Result"), this);
    auto* resultLayout = new QVBoxLayout(resultBox);
    _rbNewSegment = new QRadioButton(tr("Save as a new segment (original untouched)"), resultBox);
    _rbOverwrite = new QRadioButton(tr("Overwrite the original (disk backup saved; "
                                       "undo via right-click → Reload from backup)"), resultBox);
    resultLayout->addWidget(_rbNewSegment);
    resultLayout->addWidget(_rbOverwrite);
    if (allowInPlace && s_haveSession && s_overwrite)
        _rbOverwrite->setChecked(true);
    else
        _rbNewSegment->setChecked(true);
    _rbOverwrite->setEnabled(allowInPlace);
    if (!allowInPlace) {
        _rbNewSegment->setText(
            tr("Save as an unverified spiral hint (source untouched)"));
        _rbOverwrite->hide();
        _cbAddToFit = new QCheckBox(
            tr("Add generated hints to the current Spiral fit"), resultBox);
        _cbAddToFit->setChecked(s_addToFit);
        _cbAddToFit->setToolTip(
            tr("When a Spiral session is active, upload these patches as ephemeral, low-trust fit inputs."));
        resultLayout->addWidget(_cbAddToFit);
    }
    layout->addWidget(resultBox);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

sf_cleanup_config FiestaCleanDialog::config() const
{
    sf_cleanup_config cfg = _defaults;
    cfg.manifold_repair = _cbManifold->isChecked() ? 1 : 0;
    cfg.reorient = cfg.manifold_repair;
    cfg.fill_pinholes = _cbPinholes->isChecked() ? 1 : 0;
    cfg.sliver_cleanup = _cbSliver->isChecked() ? 1 : 0;
    cfg.cull_min_area_frac = static_cast<float>(_spCull->value());
    return cfg;
}

bool FiestaCleanDialog::overwriteOriginal() const
{
    return _rbOverwrite->isChecked() && _rbOverwrite->isEnabled();
}

QString FiestaCleanDialog::axisPointZyx() const
{
    if (!_spAxisPoint[0])
        return {};
    return QStringLiteral("%1,%2,%3")
        .arg(_spAxisPoint[0]->value(), 0, 'g', 17)
        .arg(_spAxisPoint[1]->value(), 0, 'g', 17)
        .arg(_spAxisPoint[2]->value(), 0, 'g', 17);
}

QString FiestaCleanDialog::axisDirectionZyx() const
{
    if (!_spAxisDirection[0])
        return {};
    return QStringLiteral("%1,%2,%3")
        .arg(_spAxisDirection[0]->value(), 0, 'g', 17)
        .arg(_spAxisDirection[1]->value(), 0, 'g', 17)
        .arg(_spAxisDirection[2]->value(), 0, 'g', 17);
}

double FiestaCleanDialog::wrapSpacing() const
{
    return _spWrapSpacing ? _spWrapSpacing->value() : 0.0;
}

bool FiestaCleanDialog::addToCurrentFit() const
{
    return _cbAddToFit && _cbAddToFit->isChecked();
}

void FiestaCleanDialog::accept()
{
    if (_spAxisDirection[0]) {
        const double length = std::hypot(
            _spAxisDirection[0]->value(), _spAxisDirection[1]->value(),
            _spAxisDirection[2]->value());
        if (length <= 0.0) {
            QMessageBox::warning(
                this, tr("ScrollFiesta Spiral Hints"),
                tr("Axis direction must be non-zero."));
            return;
        }
    }
    s_haveSession = true;
    s_manifold = _cbManifold->isChecked();
    s_pinholes = _cbPinholes->isChecked();
    s_sliver = _cbSliver->isChecked();
    s_cullFrac = _spCull->value();
    s_overwrite = overwriteOriginal();
    if (_spAxisPoint[0]) {
        for (size_t i = 0; i < 3; ++i) {
            s_axisPoint[i] = _spAxisPoint[i]->value();
            s_axisDirection[i] = _spAxisDirection[i]->value();
        }
        s_wrapSpacing = _spWrapSpacing->value();
        s_addToFit = _cbAddToFit->isChecked();
        s_haveHintGeometry = true;
    }
    QDialog::accept();
}

// ────────────────────────────── detangle ───────────────────────────────────

bool FiestaDetangleDialog::s_haveSession = false;
bool FiestaDetangleDialog::s_peel = true;
bool FiestaDetangleDialog::s_dev = true;
bool FiestaDetangleDialog::s_bridge = true;
bool FiestaDetangleDialog::s_overlap = true;
int FiestaDetangleDialog::s_minComp = 200;
double FiestaDetangleDialog::s_timeout = 30.0;

FiestaDetangleDialog::FiestaDetangleDialog(QWidget* parent, const sf_api* api)
    : QDialog(parent)
{
    setWindowTitle(tr("ScrollFiesta Detangle / Split"));
    _defaults = api->detangle_config_default();

    const bool peel = s_haveSession ? s_peel : _defaults.enable_peel != 0;
    const bool dev = s_haveSession ? s_dev : _defaults.enable_dev != 0;
    const bool bridge = s_haveSession ? s_bridge : _defaults.enable_bridge != 0;
    const bool overlap =
        s_haveSession ? s_overlap : _defaults.enable_overlap != 0;
    const int minComp = s_haveSession
                            ? s_minComp
                            : static_cast<int>(_defaults.min_comp_verts);
    const double timeout =
        s_haveSession ? s_timeout : _defaults.component_timeout_sec;

    auto* layout = new QVBoxLayout(this);

    auto* stages = new QGroupBox(tr("Split cascade"), this);
    auto* form = new QFormLayout(stages);
    _cbPeel = new QCheckBox(tr("Depth peel (stacked near-parallel wraps)"), stages);
    _cbPeel->setChecked(peel);
    form->addRow(_cbPeel);
    _cbDev = new QCheckBox(tr("Developability cut (fused-wrap seams; gated)"), stages);
    _cbDev->setChecked(dev);
    form->addRow(_cbDev);
    _cbBridge = new QCheckBox(tr("Bridge cut (thin necks, max-flow)"), stages);
    _cbBridge->setChecked(bridge);
    form->addRow(_cbBridge);
    _cbOverlap = new QCheckBox(tr("Overlap separation (lifted multicut)"), stages);
    _cbOverlap->setChecked(overlap);
    form->addRow(_cbOverlap);
    _spMinComp = new QSpinBox(stages);
    _spMinComp->setRange(10, 100000);
    _spMinComp->setValue(minComp);
    _spMinComp->setToolTip(tr("Pieces smaller than this many vertices are dropped as crumbs."));
    form->addRow(tr("Smallest surviving piece (vertices):"), _spMinComp);
    _spTimeout = new QDoubleSpinBox(stages);
    _spTimeout->setRange(0.0, 3600.0);
    _spTimeout->setDecimals(1);
    _spTimeout->setValue(timeout);
    _spTimeout->setToolTip(tr("Per-piece wall clock for the bridge/overlap stages. 0 = unlimited."));
    form->addRow(tr("Per-piece timeout (seconds):"), _spTimeout);
    layout->addWidget(stages);

    auto* note = new QLabel(
        tr("Pieces are always saved as new segments; the original is never touched."),
        this);
    note->setWordWrap(true);
    layout->addWidget(note);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

sf_detangle_config FiestaDetangleDialog::config() const
{
    sf_detangle_config cfg = _defaults;
    cfg.enable_peel = _cbPeel->isChecked() ? 1 : 0;
    cfg.enable_dev = _cbDev->isChecked() ? 1 : 0;
    cfg.enable_bridge = _cbBridge->isChecked() ? 1 : 0;
    cfg.enable_overlap = _cbOverlap->isChecked() ? 1 : 0;
    cfg.min_comp_verts = static_cast<size_t>(_spMinComp->value());
    cfg.component_timeout_sec = _spTimeout->value();
    return cfg;
}

void FiestaDetangleDialog::accept()
{
    s_haveSession = true;
    s_peel = _cbPeel->isChecked();
    s_dev = _cbDev->isChecked();
    s_bridge = _cbBridge->isChecked();
    s_overlap = _cbOverlap->isChecked();
    s_minComp = _spMinComp->value();
    s_timeout = _spTimeout->value();
    QDialog::accept();
}
