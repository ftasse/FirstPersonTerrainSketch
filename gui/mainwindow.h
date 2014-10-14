#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDoubleSpinBox>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
private:
    Ui::MainWindow *ui;
    QWidget *weighting_widget_;
    std::vector<QDoubleSpinBox*> weight_boxes_;

    void setupExternalConnections();

    void setupDeformationWeighting();

public slots:
    void printInfoToStatusBar (QString msg);

    bool on_actionLoadTerrain_triggered();
    bool on_actionSaveTerrain_triggered();

    bool on_actionLoadSketch_triggered();
    bool on_actionSaveSketch_triggered();

    bool on_actionComputeNoiseStats_triggered();

    void on_resetCameraPushButton_clicked(bool status);

    void on_wireframemodeCheckBox_toggled(bool status);
    void on_cameramodeCheckBox_toggled(bool status);

    void on_deformTerrainPushButton_clicked(bool status);
    void on_lowerSilhouettesPushButton_clicked(bool status);
    void on_detectSilhouettesPushButton_clicked(bool status);
    void on_detectRidgesPushButton_clicked(bool status);
    void on_detectFeaturesPushButton_clicked(bool status);
    void on_embedSketchPushButton_clicked(bool status);

    void on_matchPushButton_clicked(bool status);

    void on_addSketchPushButton_clicked(bool status);
    void on_deleteSelectedPushButton_clicked(bool status);

    void on_personheightSlider_valueChanged(int val);

    void updateDeformationWeighting();
};

#endif // MAINWINDOW_H
