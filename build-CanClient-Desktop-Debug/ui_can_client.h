/********************************************************************************
** Form generated from reading UI file 'can_client.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CAN_CLIENT_H
#define UI_CAN_CLIENT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CanClient
{
public:
    QWidget *centralWidget;
    QWidget *formLayoutWidget;
    QFormLayout *formLayout;
    QTabWidget *devicesTab;
    QWidget *hydrophones;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout_4;
    QLabel *label_13;
    QSpinBox *spinBox_Hydr_Acq_Thrs;
    QSpinBox *spinBox_Hydr_Acq_Th_Mode;
    QLabel *label_11;
    QSpinBox *spinBox_Hydr_Samp_Count;
    QLabel *label_Hydr_Phase_Calc_Alg;
    QLabel *label_Hydr_Ping_Freq;
    QSpinBox *spinBox_Hydr_Phase_Calc_Alg;
    QLabel *label_Hydr_Gain;
    QLabel *label_14;
    QLabel *label_9;
    QSpinBox *spinBox_Hydr_Filt_Thrs;
    QLabel *label_Hydr_Filt_Thrs;
    QLabel *label_10;
    QLabel *label_16;
    QLabel *label_12;
    QSpinBox *spinBox_Hydr_Cont_F_Freq;
    QLabel *label_Hydr_Acq_Thrs;
    QSpinBox *spinBox_Hydr_Gain;
    QLabel *label_18;
    QSpinBox *spinBox_Hydr_Pinger_Freq;
    QLabel *label_Hydr_Cont_F_Freq;
    QLabel *label_Hydr_Samp_Count;
    QLabel *label_Hydr_Acq_Th_Mode;
    QWidget *gridLayoutWidget_4;
    QGridLayout *gridLayout_5;
    QLabel *label_Hydr_Bw;
    QSpinBox *spinBox_Hydr_Fft_Prefilter;
    QLabel *label_15;
    QLabel *label_20;
    QSpinBox *spinBox_Hydr_Bw;
    QLabel *label_Hydr_Preamp_Gain;
    QLabel *label_Hydr_Fft_Thrs;
    QLabel *label_21;
    QLabel *label_22;
    QSpinBox *spinBox_Hydr_Fft_Prefilter_T;
    QLabel *label_Hydr_Fft_Prefilter_T;
    QLabel *label_23;
    QLabel *label_24;
    QLabel *label_25;
    QSpinBox *spinBox_Hydr_Cont_Fil_Freq;
    QLabel *label_Hydr_Fft_Prefilter;
    QSpinBox *spinBox_Hydr_Preamp_Gain;
    QSpinBox *spinBox_Hydr_Fft_Thrs;
    QLabel *label_Hydr_Cont_Fil_Freq;
    QSpinBox *spinBox_Hydr_Fft_Trig_Mode;
    QLabel *label_17;
    QSpinBox *spinBox_Hydr_Cutoff;
    QLabel *label_Hydr_Fft_Trig_Mode;
    QLabel *label_Hydr_Cutoff;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *pushButton_En_Hydros;
    QPushButton *pushButton_En_Wave;
    QPushButton *pushButton_En_Fft;
    QWidget *thrusters;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QPushButton *pushButton_Thruster_Left;
    QPushButton *pushButton_Thruster_Down;
    QPushButton *pushButton_Thruster_Right;
    QPushButton *pushButton_Thruster_For;
    QPushButton *pushButton_Thruster_Back;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton_Thruster_Up;
    QPushButton *pushButton_Thruster_Stop;
    QPushButton *pushButton_Thruster_Rot_L;
    QPushButton *pushButton_Thruster_Rot_R;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_3;
    QSpinBox *spinBox_Thruster_Port;
    QSpinBox *spinBox_Thruster_Front_Heading;
    QLabel *label_6;
    QSpinBox *spinBox_Thruster_Back_Heading;
    QSpinBox *spinBox_Thruster_Starboard;
    QSpinBox *spinBox_Thruster_Back_Depth;
    QLabel *label;
    QLabel *label_4;
    QSpinBox *spinBox_Thruster_Front_Depth;
    QLabel *label_2;
    QLabel *label_5;
    QLabel *label_3;
    QLabel *label_Thruster_Speed_Bd;
    QLabel *label_Thruster_Speed_Bh;
    QLabel *label_Thruster_Speed_Fh;
    QLabel *label_Thruster_Speed_Fd;
    QLabel *label_Thruster_Speed_P;
    QLabel *label_Thruster_Speed_S;
    QPushButton *pushButton_Thruster_Speed_Bd;
    QPushButton *pushButton_Thruster_Speed_Bh;
    QPushButton *pushButton_Thruster_Speed_Fd;
    QPushButton *pushButton_Thruster_Speed_P;
    QPushButton *pushButton_Thruster_Speed_Fh;
    QPushButton *pushButton_Thruster_Speed_S;
    QLabel *label_7;
    QLabel *label_8;
    QWidget *barometer;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_19;
    QLabel *label_Baro_press;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_26;
    QLabel *label_Baro_press_2;
    QWidget *torpedo_launchers;
    QWidget *grabber;
    QWidget *droppers;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *CanClient)
    {
        if (CanClient->objectName().isEmpty())
            CanClient->setObjectName(QStringLiteral("CanClient"));
        CanClient->resize(919, 437);
        centralWidget = new QWidget(CanClient);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        formLayoutWidget = new QWidget(centralWidget);
        formLayoutWidget->setObjectName(QStringLiteral("formLayoutWidget"));
        formLayoutWidget->setGeometry(QRect(0, 0, 911, 802));
        formLayout = new QFormLayout(formLayoutWidget);
        formLayout->setSpacing(6);
        formLayout->setContentsMargins(11, 11, 11, 11);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        devicesTab = new QTabWidget(formLayoutWidget);
        devicesTab->setObjectName(QStringLiteral("devicesTab"));
        devicesTab->setMinimumSize(QSize(800, 800));
        hydrophones = new QWidget();
        hydrophones->setObjectName(QStringLiteral("hydrophones"));
        gridLayoutWidget_3 = new QWidget(hydrophones);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(10, 10, 421, 260));
        gridLayout_4 = new QGridLayout(gridLayoutWidget_3);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        label_13 = new QLabel(gridLayoutWidget_3);
        label_13->setObjectName(QStringLiteral("label_13"));

        gridLayout_4->addWidget(label_13, 1, 0, 1, 1);

        spinBox_Hydr_Acq_Thrs = new QSpinBox(gridLayoutWidget_3);
        spinBox_Hydr_Acq_Thrs->setObjectName(QStringLiteral("spinBox_Hydr_Acq_Thrs"));
        spinBox_Hydr_Acq_Thrs->setMaximum(2000000000);
        spinBox_Hydr_Acq_Thrs->setSingleStep(1000000);
        spinBox_Hydr_Acq_Thrs->setValue(1124800758);

        gridLayout_4->addWidget(spinBox_Hydr_Acq_Thrs, 2, 1, 1, 1);

        spinBox_Hydr_Acq_Th_Mode = new QSpinBox(gridLayoutWidget_3);
        spinBox_Hydr_Acq_Th_Mode->setObjectName(QStringLiteral("spinBox_Hydr_Acq_Th_Mode"));
        spinBox_Hydr_Acq_Th_Mode->setMaximum(5);
        spinBox_Hydr_Acq_Th_Mode->setValue(1);

        gridLayout_4->addWidget(spinBox_Hydr_Acq_Th_Mode, 6, 1, 1, 1);

        label_11 = new QLabel(gridLayoutWidget_3);
        label_11->setObjectName(QStringLiteral("label_11"));

        gridLayout_4->addWidget(label_11, 2, 0, 1, 1);

        spinBox_Hydr_Samp_Count = new QSpinBox(gridLayoutWidget_3);
        spinBox_Hydr_Samp_Count->setObjectName(QStringLiteral("spinBox_Hydr_Samp_Count"));
        spinBox_Hydr_Samp_Count->setMaximum(1000);
        spinBox_Hydr_Samp_Count->setValue(256);

        gridLayout_4->addWidget(spinBox_Hydr_Samp_Count, 5, 1, 1, 1);

        label_Hydr_Phase_Calc_Alg = new QLabel(gridLayoutWidget_3);
        label_Hydr_Phase_Calc_Alg->setObjectName(QStringLiteral("label_Hydr_Phase_Calc_Alg"));

        gridLayout_4->addWidget(label_Hydr_Phase_Calc_Alg, 7, 2, 1, 1);

        label_Hydr_Ping_Freq = new QLabel(gridLayoutWidget_3);
        label_Hydr_Ping_Freq->setObjectName(QStringLiteral("label_Hydr_Ping_Freq"));

        gridLayout_4->addWidget(label_Hydr_Ping_Freq, 0, 2, 1, 1);

        spinBox_Hydr_Phase_Calc_Alg = new QSpinBox(gridLayoutWidget_3);
        spinBox_Hydr_Phase_Calc_Alg->setObjectName(QStringLiteral("spinBox_Hydr_Phase_Calc_Alg"));
        spinBox_Hydr_Phase_Calc_Alg->setMaximum(1);

        gridLayout_4->addWidget(spinBox_Hydr_Phase_Calc_Alg, 7, 1, 1, 1);

        label_Hydr_Gain = new QLabel(gridLayoutWidget_3);
        label_Hydr_Gain->setObjectName(QStringLiteral("label_Hydr_Gain"));

        gridLayout_4->addWidget(label_Hydr_Gain, 1, 2, 1, 1);

        label_14 = new QLabel(gridLayoutWidget_3);
        label_14->setObjectName(QStringLiteral("label_14"));

        gridLayout_4->addWidget(label_14, 3, 0, 1, 1);

        label_9 = new QLabel(gridLayoutWidget_3);
        label_9->setObjectName(QStringLiteral("label_9"));

        gridLayout_4->addWidget(label_9, 0, 0, 1, 1);

        spinBox_Hydr_Filt_Thrs = new QSpinBox(gridLayoutWidget_3);
        spinBox_Hydr_Filt_Thrs->setObjectName(QStringLiteral("spinBox_Hydr_Filt_Thrs"));
        spinBox_Hydr_Filt_Thrs->setValue(15);

        gridLayout_4->addWidget(spinBox_Hydr_Filt_Thrs, 3, 1, 1, 1);

        label_Hydr_Filt_Thrs = new QLabel(gridLayoutWidget_3);
        label_Hydr_Filt_Thrs->setObjectName(QStringLiteral("label_Hydr_Filt_Thrs"));

        gridLayout_4->addWidget(label_Hydr_Filt_Thrs, 3, 2, 1, 1);

        label_10 = new QLabel(gridLayoutWidget_3);
        label_10->setObjectName(QStringLiteral("label_10"));

        gridLayout_4->addWidget(label_10, 5, 0, 1, 1);

        label_16 = new QLabel(gridLayoutWidget_3);
        label_16->setObjectName(QStringLiteral("label_16"));

        gridLayout_4->addWidget(label_16, 6, 0, 1, 1);

        label_12 = new QLabel(gridLayoutWidget_3);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout_4->addWidget(label_12, 4, 0, 1, 1);

        spinBox_Hydr_Cont_F_Freq = new QSpinBox(gridLayoutWidget_3);
        spinBox_Hydr_Cont_F_Freq->setObjectName(QStringLiteral("spinBox_Hydr_Cont_F_Freq"));
        spinBox_Hydr_Cont_F_Freq->setSingleStep(1);
        spinBox_Hydr_Cont_F_Freq->setValue(5);

        gridLayout_4->addWidget(spinBox_Hydr_Cont_F_Freq, 4, 1, 1, 1);

        label_Hydr_Acq_Thrs = new QLabel(gridLayoutWidget_3);
        label_Hydr_Acq_Thrs->setObjectName(QStringLiteral("label_Hydr_Acq_Thrs"));

        gridLayout_4->addWidget(label_Hydr_Acq_Thrs, 2, 2, 1, 1);

        spinBox_Hydr_Gain = new QSpinBox(gridLayoutWidget_3);
        spinBox_Hydr_Gain->setObjectName(QStringLiteral("spinBox_Hydr_Gain"));
        spinBox_Hydr_Gain->setValue(4);

        gridLayout_4->addWidget(spinBox_Hydr_Gain, 1, 1, 1, 1);

        label_18 = new QLabel(gridLayoutWidget_3);
        label_18->setObjectName(QStringLiteral("label_18"));

        gridLayout_4->addWidget(label_18, 7, 0, 1, 1);

        spinBox_Hydr_Pinger_Freq = new QSpinBox(gridLayoutWidget_3);
        spinBox_Hydr_Pinger_Freq->setObjectName(QStringLiteral("spinBox_Hydr_Pinger_Freq"));
        spinBox_Hydr_Pinger_Freq->setMinimum(10);
        spinBox_Hydr_Pinger_Freq->setMaximum(60);
        spinBox_Hydr_Pinger_Freq->setValue(25);

        gridLayout_4->addWidget(spinBox_Hydr_Pinger_Freq, 0, 1, 1, 1);

        label_Hydr_Cont_F_Freq = new QLabel(gridLayoutWidget_3);
        label_Hydr_Cont_F_Freq->setObjectName(QStringLiteral("label_Hydr_Cont_F_Freq"));

        gridLayout_4->addWidget(label_Hydr_Cont_F_Freq, 4, 2, 1, 1);

        label_Hydr_Samp_Count = new QLabel(gridLayoutWidget_3);
        label_Hydr_Samp_Count->setObjectName(QStringLiteral("label_Hydr_Samp_Count"));

        gridLayout_4->addWidget(label_Hydr_Samp_Count, 5, 2, 1, 1);

        label_Hydr_Acq_Th_Mode = new QLabel(gridLayoutWidget_3);
        label_Hydr_Acq_Th_Mode->setObjectName(QStringLiteral("label_Hydr_Acq_Th_Mode"));

        gridLayout_4->addWidget(label_Hydr_Acq_Th_Mode, 6, 2, 1, 1);

        gridLayoutWidget_4 = new QWidget(hydrophones);
        gridLayoutWidget_4->setObjectName(QStringLiteral("gridLayoutWidget_4"));
        gridLayoutWidget_4->setGeometry(QRect(480, 10, 421, 260));
        gridLayout_5 = new QGridLayout(gridLayoutWidget_4);
        gridLayout_5->setSpacing(6);
        gridLayout_5->setContentsMargins(11, 11, 11, 11);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        label_Hydr_Bw = new QLabel(gridLayoutWidget_4);
        label_Hydr_Bw->setObjectName(QStringLiteral("label_Hydr_Bw"));

        gridLayout_5->addWidget(label_Hydr_Bw, 5, 2, 1, 1);

        spinBox_Hydr_Fft_Prefilter = new QSpinBox(gridLayoutWidget_4);
        spinBox_Hydr_Fft_Prefilter->setObjectName(QStringLiteral("spinBox_Hydr_Fft_Prefilter"));
        spinBox_Hydr_Fft_Prefilter->setMaximum(5);
        spinBox_Hydr_Fft_Prefilter->setValue(1);

        gridLayout_5->addWidget(spinBox_Hydr_Fft_Prefilter, 2, 1, 1, 1);

        label_15 = new QLabel(gridLayoutWidget_4);
        label_15->setObjectName(QStringLiteral("label_15"));

        gridLayout_5->addWidget(label_15, 1, 0, 1, 1);

        label_20 = new QLabel(gridLayoutWidget_4);
        label_20->setObjectName(QStringLiteral("label_20"));

        gridLayout_5->addWidget(label_20, 2, 0, 1, 1);

        spinBox_Hydr_Bw = new QSpinBox(gridLayoutWidget_4);
        spinBox_Hydr_Bw->setObjectName(QStringLiteral("spinBox_Hydr_Bw"));
        spinBox_Hydr_Bw->setValue(1);

        gridLayout_5->addWidget(spinBox_Hydr_Bw, 5, 1, 1, 1);

        label_Hydr_Preamp_Gain = new QLabel(gridLayoutWidget_4);
        label_Hydr_Preamp_Gain->setObjectName(QStringLiteral("label_Hydr_Preamp_Gain"));

        gridLayout_5->addWidget(label_Hydr_Preamp_Gain, 0, 2, 1, 1);

        label_Hydr_Fft_Thrs = new QLabel(gridLayoutWidget_4);
        label_Hydr_Fft_Thrs->setObjectName(QStringLiteral("label_Hydr_Fft_Thrs"));

        gridLayout_5->addWidget(label_Hydr_Fft_Thrs, 1, 2, 1, 1);

        label_21 = new QLabel(gridLayoutWidget_4);
        label_21->setObjectName(QStringLiteral("label_21"));

        gridLayout_5->addWidget(label_21, 3, 0, 1, 1);

        label_22 = new QLabel(gridLayoutWidget_4);
        label_22->setObjectName(QStringLiteral("label_22"));

        gridLayout_5->addWidget(label_22, 0, 0, 1, 1);

        spinBox_Hydr_Fft_Prefilter_T = new QSpinBox(gridLayoutWidget_4);
        spinBox_Hydr_Fft_Prefilter_T->setObjectName(QStringLiteral("spinBox_Hydr_Fft_Prefilter_T"));
        spinBox_Hydr_Fft_Prefilter_T->setMaximum(5);
        spinBox_Hydr_Fft_Prefilter_T->setValue(5);

        gridLayout_5->addWidget(spinBox_Hydr_Fft_Prefilter_T, 3, 1, 1, 1);

        label_Hydr_Fft_Prefilter_T = new QLabel(gridLayoutWidget_4);
        label_Hydr_Fft_Prefilter_T->setObjectName(QStringLiteral("label_Hydr_Fft_Prefilter_T"));

        gridLayout_5->addWidget(label_Hydr_Fft_Prefilter_T, 3, 2, 1, 1);

        label_23 = new QLabel(gridLayoutWidget_4);
        label_23->setObjectName(QStringLiteral("label_23"));

        gridLayout_5->addWidget(label_23, 5, 0, 1, 1);

        label_24 = new QLabel(gridLayoutWidget_4);
        label_24->setObjectName(QStringLiteral("label_24"));

        gridLayout_5->addWidget(label_24, 6, 0, 1, 1);

        label_25 = new QLabel(gridLayoutWidget_4);
        label_25->setObjectName(QStringLiteral("label_25"));

        gridLayout_5->addWidget(label_25, 4, 0, 1, 1);

        spinBox_Hydr_Cont_Fil_Freq = new QSpinBox(gridLayoutWidget_4);
        spinBox_Hydr_Cont_Fil_Freq->setObjectName(QStringLiteral("spinBox_Hydr_Cont_Fil_Freq"));
        spinBox_Hydr_Cont_Fil_Freq->setValue(5);

        gridLayout_5->addWidget(spinBox_Hydr_Cont_Fil_Freq, 4, 1, 1, 1);

        label_Hydr_Fft_Prefilter = new QLabel(gridLayoutWidget_4);
        label_Hydr_Fft_Prefilter->setObjectName(QStringLiteral("label_Hydr_Fft_Prefilter"));

        gridLayout_5->addWidget(label_Hydr_Fft_Prefilter, 2, 2, 1, 1);

        spinBox_Hydr_Preamp_Gain = new QSpinBox(gridLayoutWidget_4);
        spinBox_Hydr_Preamp_Gain->setObjectName(QStringLiteral("spinBox_Hydr_Preamp_Gain"));
        spinBox_Hydr_Preamp_Gain->setValue(1);

        gridLayout_5->addWidget(spinBox_Hydr_Preamp_Gain, 0, 1, 1, 1);

        spinBox_Hydr_Fft_Thrs = new QSpinBox(gridLayoutWidget_4);
        spinBox_Hydr_Fft_Thrs->setObjectName(QStringLiteral("spinBox_Hydr_Fft_Thrs"));
        spinBox_Hydr_Fft_Thrs->setMaximum(50000);
        spinBox_Hydr_Fft_Thrs->setValue(38000);

        gridLayout_5->addWidget(spinBox_Hydr_Fft_Thrs, 1, 1, 1, 1);

        label_Hydr_Cont_Fil_Freq = new QLabel(gridLayoutWidget_4);
        label_Hydr_Cont_Fil_Freq->setObjectName(QStringLiteral("label_Hydr_Cont_Fil_Freq"));

        gridLayout_5->addWidget(label_Hydr_Cont_Fil_Freq, 4, 2, 1, 1);

        spinBox_Hydr_Fft_Trig_Mode = new QSpinBox(gridLayoutWidget_4);
        spinBox_Hydr_Fft_Trig_Mode->setObjectName(QStringLiteral("spinBox_Hydr_Fft_Trig_Mode"));
        spinBox_Hydr_Fft_Trig_Mode->setMaximum(5);
        spinBox_Hydr_Fft_Trig_Mode->setValue(1);

        gridLayout_5->addWidget(spinBox_Hydr_Fft_Trig_Mode, 6, 1, 1, 1);

        label_17 = new QLabel(gridLayoutWidget_4);
        label_17->setObjectName(QStringLiteral("label_17"));

        gridLayout_5->addWidget(label_17, 7, 0, 1, 1);

        spinBox_Hydr_Cutoff = new QSpinBox(gridLayoutWidget_4);
        spinBox_Hydr_Cutoff->setObjectName(QStringLiteral("spinBox_Hydr_Cutoff"));
        spinBox_Hydr_Cutoff->setValue(24);

        gridLayout_5->addWidget(spinBox_Hydr_Cutoff, 7, 1, 1, 1);

        label_Hydr_Fft_Trig_Mode = new QLabel(gridLayoutWidget_4);
        label_Hydr_Fft_Trig_Mode->setObjectName(QStringLiteral("label_Hydr_Fft_Trig_Mode"));

        gridLayout_5->addWidget(label_Hydr_Fft_Trig_Mode, 6, 2, 1, 1);

        label_Hydr_Cutoff = new QLabel(gridLayoutWidget_4);
        label_Hydr_Cutoff->setObjectName(QStringLiteral("label_Hydr_Cutoff"));

        gridLayout_5->addWidget(label_Hydr_Cutoff, 7, 2, 1, 1);

        horizontalLayoutWidget = new QWidget(hydrophones);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 290, 891, 31));
        horizontalLayout_3 = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        pushButton_En_Hydros = new QPushButton(horizontalLayoutWidget);
        pushButton_En_Hydros->setObjectName(QStringLiteral("pushButton_En_Hydros"));

        horizontalLayout_3->addWidget(pushButton_En_Hydros);

        pushButton_En_Wave = new QPushButton(horizontalLayoutWidget);
        pushButton_En_Wave->setObjectName(QStringLiteral("pushButton_En_Wave"));

        horizontalLayout_3->addWidget(pushButton_En_Wave);

        pushButton_En_Fft = new QPushButton(horizontalLayoutWidget);
        pushButton_En_Fft->setObjectName(QStringLiteral("pushButton_En_Fft"));

        horizontalLayout_3->addWidget(pushButton_En_Fft);

        devicesTab->addTab(hydrophones, QString());
        thrusters = new QWidget();
        thrusters->setObjectName(QStringLiteral("thrusters"));
        gridLayoutWidget = new QWidget(thrusters);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(20, 50, 391, 221));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_Thruster_Left = new QPushButton(gridLayoutWidget);
        pushButton_Thruster_Left->setObjectName(QStringLiteral("pushButton_Thruster_Left"));

        gridLayout->addWidget(pushButton_Thruster_Left, 2, 0, 1, 1);

        pushButton_Thruster_Down = new QPushButton(gridLayoutWidget);
        pushButton_Thruster_Down->setObjectName(QStringLiteral("pushButton_Thruster_Down"));

        gridLayout->addWidget(pushButton_Thruster_Down, 3, 3, 1, 1);

        pushButton_Thruster_Right = new QPushButton(gridLayoutWidget);
        pushButton_Thruster_Right->setObjectName(QStringLiteral("pushButton_Thruster_Right"));

        gridLayout->addWidget(pushButton_Thruster_Right, 2, 2, 1, 1);

        pushButton_Thruster_For = new QPushButton(gridLayoutWidget);
        pushButton_Thruster_For->setObjectName(QStringLiteral("pushButton_Thruster_For"));

        gridLayout->addWidget(pushButton_Thruster_For, 0, 1, 1, 1);

        pushButton_Thruster_Back = new QPushButton(gridLayoutWidget);
        pushButton_Thruster_Back->setObjectName(QStringLiteral("pushButton_Thruster_Back"));

        gridLayout->addWidget(pushButton_Thruster_Back, 3, 1, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 2, 3, 1, 1);

        pushButton_Thruster_Up = new QPushButton(gridLayoutWidget);
        pushButton_Thruster_Up->setObjectName(QStringLiteral("pushButton_Thruster_Up"));

        gridLayout->addWidget(pushButton_Thruster_Up, 0, 3, 1, 1);

        pushButton_Thruster_Stop = new QPushButton(gridLayoutWidget);
        pushButton_Thruster_Stop->setObjectName(QStringLiteral("pushButton_Thruster_Stop"));
        QPalette palette;
        QBrush brush(QColor(0, 0, 0, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(242, 0, 4, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        QBrush brush2(QColor(255, 108, 110, 255));
        brush2.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Light, brush2);
        QBrush brush3(QColor(248, 54, 57, 255));
        brush3.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Midlight, brush3);
        QBrush brush4(QColor(121, 0, 2, 255));
        brush4.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Dark, brush4);
        QBrush brush5(QColor(161, 0, 2, 255));
        brush5.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        QBrush brush6(QColor(255, 255, 255, 255));
        brush6.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Active, QPalette::Base, brush6);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Active, QPalette::Shadow, brush);
        QBrush brush7(QColor(248, 127, 129, 255));
        brush7.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::AlternateBase, brush7);
        QBrush brush8(QColor(255, 255, 220, 255));
        brush8.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Active, QPalette::ToolTipText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Light, brush2);
        palette.setBrush(QPalette::Inactive, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Inactive, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush7);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Light, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Disabled, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush);
        pushButton_Thruster_Stop->setPalette(palette);

        gridLayout->addWidget(pushButton_Thruster_Stop, 2, 1, 1, 1);

        pushButton_Thruster_Rot_L = new QPushButton(gridLayoutWidget);
        pushButton_Thruster_Rot_L->setObjectName(QStringLiteral("pushButton_Thruster_Rot_L"));

        gridLayout->addWidget(pushButton_Thruster_Rot_L, 0, 0, 1, 1);

        pushButton_Thruster_Rot_R = new QPushButton(gridLayoutWidget);
        pushButton_Thruster_Rot_R->setObjectName(QStringLiteral("pushButton_Thruster_Rot_R"));

        gridLayout->addWidget(pushButton_Thruster_Rot_R, 0, 2, 1, 1);

        gridLayoutWidget_2 = new QWidget(thrusters);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(450, 50, 391, 221));
        gridLayout_3 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        spinBox_Thruster_Port = new QSpinBox(gridLayoutWidget_2);
        spinBox_Thruster_Port->setObjectName(QStringLiteral("spinBox_Thruster_Port"));

        gridLayout_3->addWidget(spinBox_Thruster_Port, 4, 1, 1, 1);

        spinBox_Thruster_Front_Heading = new QSpinBox(gridLayoutWidget_2);
        spinBox_Thruster_Front_Heading->setObjectName(QStringLiteral("spinBox_Thruster_Front_Heading"));

        gridLayout_3->addWidget(spinBox_Thruster_Front_Heading, 3, 1, 1, 1);

        label_6 = new QLabel(gridLayoutWidget_2);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout_3->addWidget(label_6, 0, 0, 1, 1);

        spinBox_Thruster_Back_Heading = new QSpinBox(gridLayoutWidget_2);
        spinBox_Thruster_Back_Heading->setObjectName(QStringLiteral("spinBox_Thruster_Back_Heading"));

        gridLayout_3->addWidget(spinBox_Thruster_Back_Heading, 1, 1, 1, 1);

        spinBox_Thruster_Starboard = new QSpinBox(gridLayoutWidget_2);
        spinBox_Thruster_Starboard->setObjectName(QStringLiteral("spinBox_Thruster_Starboard"));

        gridLayout_3->addWidget(spinBox_Thruster_Starboard, 5, 1, 1, 1);

        spinBox_Thruster_Back_Depth = new QSpinBox(gridLayoutWidget_2);
        spinBox_Thruster_Back_Depth->setObjectName(QStringLiteral("spinBox_Thruster_Back_Depth"));

        gridLayout_3->addWidget(spinBox_Thruster_Back_Depth, 0, 1, 1, 1);

        label = new QLabel(gridLayoutWidget_2);
        label->setObjectName(QStringLiteral("label"));

        gridLayout_3->addWidget(label, 5, 0, 1, 1);

        label_4 = new QLabel(gridLayoutWidget_2);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout_3->addWidget(label_4, 2, 0, 1, 1);

        spinBox_Thruster_Front_Depth = new QSpinBox(gridLayoutWidget_2);
        spinBox_Thruster_Front_Depth->setObjectName(QStringLiteral("spinBox_Thruster_Front_Depth"));

        gridLayout_3->addWidget(spinBox_Thruster_Front_Depth, 2, 1, 1, 1);

        label_2 = new QLabel(gridLayoutWidget_2);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout_3->addWidget(label_2, 4, 0, 1, 1);

        label_5 = new QLabel(gridLayoutWidget_2);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout_3->addWidget(label_5, 1, 0, 1, 1);

        label_3 = new QLabel(gridLayoutWidget_2);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout_3->addWidget(label_3, 3, 0, 1, 1);

        label_Thruster_Speed_Bd = new QLabel(gridLayoutWidget_2);
        label_Thruster_Speed_Bd->setObjectName(QStringLiteral("label_Thruster_Speed_Bd"));

        gridLayout_3->addWidget(label_Thruster_Speed_Bd, 0, 2, 1, 1);

        label_Thruster_Speed_Bh = new QLabel(gridLayoutWidget_2);
        label_Thruster_Speed_Bh->setObjectName(QStringLiteral("label_Thruster_Speed_Bh"));

        gridLayout_3->addWidget(label_Thruster_Speed_Bh, 1, 2, 1, 1);

        label_Thruster_Speed_Fh = new QLabel(gridLayoutWidget_2);
        label_Thruster_Speed_Fh->setObjectName(QStringLiteral("label_Thruster_Speed_Fh"));

        gridLayout_3->addWidget(label_Thruster_Speed_Fh, 3, 2, 1, 1);

        label_Thruster_Speed_Fd = new QLabel(gridLayoutWidget_2);
        label_Thruster_Speed_Fd->setObjectName(QStringLiteral("label_Thruster_Speed_Fd"));

        gridLayout_3->addWidget(label_Thruster_Speed_Fd, 2, 2, 1, 1);

        label_Thruster_Speed_P = new QLabel(gridLayoutWidget_2);
        label_Thruster_Speed_P->setObjectName(QStringLiteral("label_Thruster_Speed_P"));

        gridLayout_3->addWidget(label_Thruster_Speed_P, 4, 2, 1, 1);

        label_Thruster_Speed_S = new QLabel(gridLayoutWidget_2);
        label_Thruster_Speed_S->setObjectName(QStringLiteral("label_Thruster_Speed_S"));

        gridLayout_3->addWidget(label_Thruster_Speed_S, 5, 2, 1, 1);

        pushButton_Thruster_Speed_Bd = new QPushButton(gridLayoutWidget_2);
        pushButton_Thruster_Speed_Bd->setObjectName(QStringLiteral("pushButton_Thruster_Speed_Bd"));

        gridLayout_3->addWidget(pushButton_Thruster_Speed_Bd, 0, 3, 1, 1);

        pushButton_Thruster_Speed_Bh = new QPushButton(gridLayoutWidget_2);
        pushButton_Thruster_Speed_Bh->setObjectName(QStringLiteral("pushButton_Thruster_Speed_Bh"));

        gridLayout_3->addWidget(pushButton_Thruster_Speed_Bh, 1, 3, 1, 1);

        pushButton_Thruster_Speed_Fd = new QPushButton(gridLayoutWidget_2);
        pushButton_Thruster_Speed_Fd->setObjectName(QStringLiteral("pushButton_Thruster_Speed_Fd"));

        gridLayout_3->addWidget(pushButton_Thruster_Speed_Fd, 2, 3, 1, 1);

        pushButton_Thruster_Speed_P = new QPushButton(gridLayoutWidget_2);
        pushButton_Thruster_Speed_P->setObjectName(QStringLiteral("pushButton_Thruster_Speed_P"));

        gridLayout_3->addWidget(pushButton_Thruster_Speed_P, 4, 3, 1, 1);

        pushButton_Thruster_Speed_Fh = new QPushButton(gridLayoutWidget_2);
        pushButton_Thruster_Speed_Fh->setObjectName(QStringLiteral("pushButton_Thruster_Speed_Fh"));

        gridLayout_3->addWidget(pushButton_Thruster_Speed_Fh, 3, 3, 1, 1);

        pushButton_Thruster_Speed_S = new QPushButton(gridLayoutWidget_2);
        pushButton_Thruster_Speed_S->setObjectName(QStringLiteral("pushButton_Thruster_Speed_S"));

        gridLayout_3->addWidget(pushButton_Thruster_Speed_S, 5, 3, 1, 1);

        label_7 = new QLabel(thrusters);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(560, 20, 151, 21));
        QFont font;
        font.setPointSize(16);
        font.setBold(true);
        font.setWeight(75);
        label_7->setFont(font);
        label_8 = new QLabel(thrusters);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(170, 20, 151, 21));
        label_8->setFont(font);
        devicesTab->addTab(thrusters, QString());
        barometer = new QWidget();
        barometer->setObjectName(QStringLiteral("barometer"));
        verticalLayoutWidget = new QWidget(barometer);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(180, 90, 160, 80));
        verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_19 = new QLabel(verticalLayoutWidget);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setFont(font);

        verticalLayout_3->addWidget(label_19);

        label_Baro_press = new QLabel(verticalLayoutWidget);
        label_Baro_press->setObjectName(QStringLiteral("label_Baro_press"));
        QFont font1;
        font1.setPointSize(12);
        font1.setBold(true);
        font1.setWeight(75);
        label_Baro_press->setFont(font1);

        verticalLayout_3->addWidget(label_Baro_press);

        verticalLayoutWidget_2 = new QWidget(barometer);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(560, 90, 160, 80));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        label_26 = new QLabel(verticalLayoutWidget_2);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setFont(font);

        verticalLayout_4->addWidget(label_26);

        label_Baro_press_2 = new QLabel(verticalLayoutWidget_2);
        label_Baro_press_2->setObjectName(QStringLiteral("label_Baro_press_2"));
        label_Baro_press_2->setFont(font1);

        verticalLayout_4->addWidget(label_Baro_press_2);

        devicesTab->addTab(barometer, QString());
        torpedo_launchers = new QWidget();
        torpedo_launchers->setObjectName(QStringLiteral("torpedo_launchers"));
        devicesTab->addTab(torpedo_launchers, QString());
        grabber = new QWidget();
        grabber->setObjectName(QStringLiteral("grabber"));
        devicesTab->addTab(grabber, QString());
        droppers = new QWidget();
        droppers->setObjectName(QStringLiteral("droppers"));
        devicesTab->addTab(droppers, QString());

        formLayout->setWidget(0, QFormLayout::SpanningRole, devicesTab);

        CanClient->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(CanClient);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 919, 25));
        CanClient->setMenuBar(menuBar);
        mainToolBar = new QToolBar(CanClient);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        CanClient->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(CanClient);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        CanClient->setStatusBar(statusBar);

        retranslateUi(CanClient);

        devicesTab->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(CanClient);
    } // setupUi

    void retranslateUi(QMainWindow *CanClient)
    {
        CanClient->setWindowTitle(QApplication::translate("CanClient", "MainWindow", 0));
        label_13->setText(QApplication::translate("CanClient", "Gain:", 0));
        label_11->setText(QApplication::translate("CanClient", "Acquisition threshold:", 0));
        label_Hydr_Phase_Calc_Alg->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Hydr_Ping_Freq->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Hydr_Gain->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_14->setText(QApplication::translate("CanClient", "Filter Threshold", 0));
        label_9->setText(QApplication::translate("CanClient", "Pinger Frequency:", 0));
        label_Hydr_Filt_Thrs->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_10->setText(QApplication::translate("CanClient", "Sample Count", 0));
        label_16->setText(QApplication::translate("CanClient", "Acquisition Threshold Mode:", 0));
        label_12->setText(QApplication::translate("CanClient", "Continuous Filter Frequency:", 0));
        label_Hydr_Acq_Thrs->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_18->setText(QApplication::translate("CanClient", "Phase Calculation Algorithm:", 0));
        label_Hydr_Cont_F_Freq->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Hydr_Samp_Count->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Hydr_Acq_Th_Mode->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Hydr_Bw->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_15->setText(QApplication::translate("CanClient", "FFT Threshold:", 0));
        label_20->setText(QApplication::translate("CanClient", "FFT Prefilter:", 0));
        label_Hydr_Preamp_Gain->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Hydr_Fft_Thrs->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_21->setText(QApplication::translate("CanClient", "FFT Prefilter Type:", 0));
        label_22->setText(QApplication::translate("CanClient", "Preamplification Gain:", 0));
        label_Hydr_Fft_Prefilter_T->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_23->setText(QApplication::translate("CanClient", "Bandwidth:", 0));
        label_24->setText(QApplication::translate("CanClient", "FFT Trigger Mode:", 0));
        label_25->setText(QApplication::translate("CanClient", "Continuous Filter Frequency:", 0));
        label_Hydr_Fft_Prefilter->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Hydr_Cont_Fil_Freq->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_17->setText(QApplication::translate("CanClient", "Cutoff Frequency:", 0));
        label_Hydr_Fft_Trig_Mode->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Hydr_Cutoff->setText(QApplication::translate("CanClient", "TextLabel", 0));
        pushButton_En_Hydros->setText(QApplication::translate("CanClient", "Enable Hydrophone", 0));
        pushButton_En_Wave->setText(QApplication::translate("CanClient", "Enable Wave", 0));
        pushButton_En_Fft->setText(QApplication::translate("CanClient", "Enable FFT ", 0));
        devicesTab->setTabText(devicesTab->indexOf(hydrophones), QApplication::translate("CanClient", "Hydrophones", 0));
        pushButton_Thruster_Left->setText(QApplication::translate("CanClient", "Left", 0));
        pushButton_Thruster_Down->setText(QApplication::translate("CanClient", "Down", 0));
        pushButton_Thruster_Right->setText(QApplication::translate("CanClient", "Right", 0));
        pushButton_Thruster_For->setText(QApplication::translate("CanClient", "Forward", 0));
        pushButton_Thruster_Back->setText(QApplication::translate("CanClient", "Backward", 0));
        pushButton_Thruster_Up->setText(QApplication::translate("CanClient", "Up", 0));
        pushButton_Thruster_Stop->setText(QApplication::translate("CanClient", "Stop", 0));
        pushButton_Thruster_Rot_L->setText(QApplication::translate("CanClient", "Rotate left", 0));
        pushButton_Thruster_Rot_R->setText(QApplication::translate("CanClient", "Rotate right", 0));
        label_6->setText(QApplication::translate("CanClient", "Back Depth", 0));
        label->setText(QApplication::translate("CanClient", "Starboard", 0));
        label_4->setText(QApplication::translate("CanClient", "Front Depth", 0));
        label_2->setText(QApplication::translate("CanClient", "Port", 0));
        label_5->setText(QApplication::translate("CanClient", "Back Heading", 0));
        label_3->setText(QApplication::translate("CanClient", "Front Heading", 0));
        label_Thruster_Speed_Bd->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Thruster_Speed_Bh->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Thruster_Speed_Fh->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Thruster_Speed_Fd->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Thruster_Speed_P->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_Thruster_Speed_S->setText(QApplication::translate("CanClient", "TextLabel", 0));
        pushButton_Thruster_Speed_Bd->setText(QApplication::translate("CanClient", "Set", 0));
        pushButton_Thruster_Speed_Bh->setText(QApplication::translate("CanClient", "Set", 0));
        pushButton_Thruster_Speed_Fd->setText(QApplication::translate("CanClient", "Set", 0));
        pushButton_Thruster_Speed_P->setText(QApplication::translate("CanClient", "Set", 0));
        pushButton_Thruster_Speed_Fh->setText(QApplication::translate("CanClient", "Set", 0));
        pushButton_Thruster_Speed_S->setText(QApplication::translate("CanClient", "Set", 0));
        label_7->setText(QApplication::translate("CanClient", "Motors power", 0));
        label_8->setText(QApplication::translate("CanClient", "Controls", 0));
        devicesTab->setTabText(devicesTab->indexOf(thrusters), QApplication::translate("CanClient", "Thrusters", 0));
        label_19->setText(QApplication::translate("CanClient", "Pressure", 0));
        label_Baro_press->setText(QApplication::translate("CanClient", "TextLabel", 0));
        label_26->setText(QApplication::translate("CanClient", "Depth", 0));
        label_Baro_press_2->setText(QApplication::translate("CanClient", "TextLabel", 0));
        devicesTab->setTabText(devicesTab->indexOf(barometer), QApplication::translate("CanClient", "Barometer", 0));
        devicesTab->setTabText(devicesTab->indexOf(torpedo_launchers), QApplication::translate("CanClient", "Torpedo Launchers", 0));
        devicesTab->setTabText(devicesTab->indexOf(grabber), QApplication::translate("CanClient", "Grabber", 0));
        devicesTab->setTabText(devicesTab->indexOf(droppers), QApplication::translate("CanClient", "Droppers", 0));
    } // retranslateUi

};

namespace Ui {
    class CanClient: public Ui_CanClient {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAN_CLIENT_H
