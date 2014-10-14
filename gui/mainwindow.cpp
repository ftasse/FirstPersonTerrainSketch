#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>

#include "tools/deformation_weighting.h"
#include "tools/evaluation.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setupExternalConnections();
    setupDeformationWeighting();

    ui->addSketchPushButton->setEnabled(ui->cameramodeCheckBox->isChecked());
    ui->viewer->setTreeWidget(ui->sketchesTreeWidget);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupExternalConnections()
{
    connect(ui->viewer, SIGNAL(sendStatusMessage(QString)),
            this, SLOT(printInfoToStatusBar(QString)));

    connect(ui->sketchmodeCheckBox, SIGNAL(toggled(bool)), ui->viewer, SLOT(on_sketchmodeCheckBox_toggled(bool)));
    connect(ui->cameramodeCheckBox, SIGNAL(toggled(bool)), ui->viewer, SLOT(on_cameramodeCheckBox_toggled(bool)));

}

void MainWindow::setupDeformationWeighting()
{
    //ui->tools_groupBox->setCheckable(true);
    //ui->tools_groupBox->setChecked(false);

    weighting_widget_ = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(); // Vertical
    weighting_widget_->setLayout(layout);
    ui->tools_groupBox->layout()->addWidget(weighting_widget_);

    QLabel *label = new QLabel();
    label->setText("Weights");
    layout->addWidget(label);

    QHBoxLayout *hlayout = new QHBoxLayout();
    QVBoxLayout *weights_layout = new QVBoxLayout(); // Horizontal
    QVBoxLayout *labels_layout = new QVBoxLayout(); // Horizontal
    hlayout->addLayout(labels_layout);
    hlayout->addLayout(weights_layout);
    layout->addLayout(hlayout);

    DeformationWeighting weighting = ui->viewer->weighting();
    for (int i=0 ; i < weighting.numWeights(); ++i)
    {
        QDoubleSpinBox *spinBox = new QDoubleSpinBox();
        spinBox->setDecimals(4);
        spinBox->setSingleStep(0.01);
        spinBox->setMinimum(0.0);
        spinBox->setMaximum(10.0);
        spinBox->setValue(weighting.weights[i]);
        weight_boxes_.push_back(spinBox);

        QLabel *label = new QLabel();
        label->setText(QString(weighting.getWeightTitle(i).c_str()));

        labels_layout->addWidget(label);
        weights_layout->addWidget(spinBox);


        connect(spinBox, SIGNAL(valueChanged(double)), this, SLOT(updateDeformationWeighting()));
    }
}

void MainWindow::updateDeformationWeighting()
{
    DeformationWeighting weighting;
    for (unsigned int  i=0 ;i<weight_boxes_.size(); ++i)
    {
        QDoubleSpinBox *spinBox =  weight_boxes_[i];
        weighting.weights[i] = spinBox->value();
    }
    ui->viewer->setWeighting(weighting);
}

void MainWindow::printInfoToStatusBar (QString msg)
{
    statusBar()->showMessage(msg, 3000); // 3 secs
}

bool MainWindow::on_actionLoadTerrain_triggered()
{
    QString terrain_path  = QFileDialog::getOpenFileName(this, "Open Terrain file", QString(),
                                                         "Terrains (*.ter *.tif)" );
    if (terrain_path.size() > 0)
        return ui->viewer->loadTerrain(terrain_path.toStdString().c_str());
    else
        return false;
}

bool MainWindow::on_actionSaveTerrain_triggered()
{
    QString terrain_path  = QFileDialog::getSaveFileName(this, "Save Terrain file", QString(),
                                                         "Terrains (*.ter *.tif)" );
    if (terrain_path.size() > 0)
        return ui->viewer->saveTerrain(terrain_path.toStdString().c_str());
    else
        return false;
}

bool MainWindow::on_actionLoadSketch_triggered()
{
    QStringList sketch_paths  = QFileDialog::getOpenFileNames(this, "Open Sketch files", QString(),
                                                         "Sketches (*.vect *.svg)" );
    if (sketch_paths.size() > 0)
    {
        bool success = true;
        for (int i = 0; i < sketch_paths.size(); ++i)
            if (!ui->viewer->loadSketch(sketch_paths.at(i).toStdString().c_str()))
                success = false;
        return success;
    }
    else
        return false;
}

bool MainWindow::on_actionSaveSketch_triggered()
{
    QString sketch_path  = QFileDialog::getSaveFileName(this, "Save Sketch file", QString(),
                                                         "Sketches (*.vect)" );
    if (sketch_path.toStdString().find(".vect") == std::string::npos)
        sketch_path += ".vect";

    if (sketch_path.size() > 0)
        return ui->viewer->saveSketch(sketch_path.toStdString().c_str());
    else
        return false;
}

bool MainWindow::on_actionComputeNoiseStats_triggered()
{
  std::cout << "\n";
  int nlevels = 3;
  int bsize = 64;

  std::vector<Terrain *> pyr;
  constructPyramid(ui->viewer->getTerrain(), pyr, nlevels+1);

  std::vector<float> variances = noise_variances(pyr);
  std::cout << "\nGlobal Variances:\n";
  for (unsigned int i = 0; i < variances.size(); ++i)
    {
      std::cout << variances[i] << " ";
    }
  std::cout << std::endl;

  int swidth = pyr.front()->width() / bsize;
  int sheight = pyr.front()->height() / bsize;

  float** noise_var_patches = new float* [nlevels];
  for (int k  = 0; k < nlevels; ++k) noise_var_patches[k] = new float [swidth*sheight];

  noise_variances(pyr, noise_var_patches, bsize);
  std::cout << "\nVariances:\n";
  for (unsigned int k = 0; k < nlevels; ++k)
    {
      std::cout << "Band " << k << ":\n";
      for (int i  = 0; i < swidth*sheight; ++i) std::cout << noise_var_patches[k][i] << " ";
      std::cout << "\n";
    }
  std::cout << std::endl;

  for (int k  = 0; k < nlevels; ++k) delete noise_var_patches[k];
  delete noise_var_patches; noise_var_patches = NULL;

  for (unsigned int i = 0; i < pyr.size(); ++i)
    delete pyr[i];
}

void MainWindow::on_wireframemodeCheckBox_toggled(bool status)
{
    ui->viewer->setWireframe(status);
}

void MainWindow::on_resetCameraPushButton_clicked(bool status)
{
    ui->viewer->setCurrentSketchIndex(-1);
}

void MainWindow::on_cameramodeCheckBox_toggled(bool status)
{
    ui->addSketchPushButton->setEnabled(status);
}

void MainWindow::on_deleteSelectedPushButton_clicked(bool status)
{
    ui->viewer->deleteSelected();
}

void MainWindow::on_deformTerrainPushButton_clicked(bool status)
{
    ui->viewer->deformTerrain();
}

void MainWindow::on_lowerSilhouettesPushButton_clicked(bool status)
{
    ui->viewer->lowerSilhouettes();
}

void MainWindow::on_detectSilhouettesPushButton_clicked(bool status)
{
    ui->viewer->extractSilhouetteEdges();
}

void MainWindow::on_detectRidgesPushButton_clicked(bool status)
{
    ui->viewer->extractRidges();
}

void MainWindow::on_detectFeaturesPushButton_clicked(bool status)
{
    ui->viewer->extractFeatures();
}

void MainWindow::on_matchPushButton_clicked(bool status)
{
  ui->viewer->matchStrokesToFeatures();
}

void MainWindow::on_embedSketchPushButton_clicked(bool status)
{
  ui->viewer->embedSketches();
}

void MainWindow::on_personheightSlider_valueChanged(int val)
{
    ui->viewer->setPersonHeight(val);
    ui->personheightLabel->setText(QString("Person Height: %1").arg(val));
}


void MainWindow::on_addSketchPushButton_clicked(bool status)
{
    QString text = ui->addSketchPushButton->text();
    if (text.contains("add", Qt::CaseInsensitive))
    {
        ui->viewer->startNewSketch();
        ui->addSketchPushButton->setText("Stop sketch");
    } else
    {
        ui->viewer->stopNewSketch();
        ui->addSketchPushButton->setText("Add sketch");
    }
}
