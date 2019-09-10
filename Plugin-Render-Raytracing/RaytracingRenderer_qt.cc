/** @file
 *
 *  Contains definitions of the ToonRenderer Renderer that require qt headers
 *  which are incompatible with glew.h.
 */

#include "RaytracingRenderer.hh"

#include <QDialog>
#include <QLabel>
#include <QSlider>
#include <QVBoxLayout>
#include <ACG/QtWidgets/QtColorChooserButton.hh>

void RaytracingRenderer::actionDialog( bool )
{
  //generate widget
  QDialog* optionsDlg = new QDialog();
  QVBoxLayout* layout = new QVBoxLayout();
  layout->setAlignment(Qt::AlignTop);

  QColor curColor;
  curColor.setRgbF(outlineCol_[0],outlineCol_[1],outlineCol_[2]);

  QLabel* label = new QLabel(tr("Palette Size [0, 10]:"));
  layout->addWidget(label);

  QSlider* paletteSizeSlider = new QSlider(Qt::Horizontal);
  paletteSizeSlider->setRange(0, 1000);
  paletteSizeSlider->setValue(int(paletteSize_ * 100.0));
  paletteSizeSlider->setTracking(true);
  layout->addWidget(paletteSizeSlider);

  QtColorChooserButton* outlineColorBtn = new QtColorChooserButton("Outline Color");
  layout->addWidget(outlineColorBtn);

  outlineColorBtn->setColor( curColor );

  optionsDlg->setLayout(layout);


  connect(paletteSizeSlider, SIGNAL(valueChanged(int)), this, SLOT(paletteSizeChanged(int)));
  connect(outlineColorBtn, SIGNAL(colorChanged(QColor)), this, SLOT(outlineColorChanged(QColor)));


  optionsDlg->show();
}
