/********************************************************************************
** Form generated from reading UI file 'alphabot_gui.ui'
**
** Created by: Qt User Interface Compiler version 5.15.13
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ALPHABOT_GUI_H
#define UI_ALPHABOT_GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QTabWidget *tabWidget;
    QWidget *tab;
    QSplitter *splitter;
    QLabel *label_2;
    QSlider *HSSpeed;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QPushButton *PBForward;
    QPushButton *PBRight;
    QPushButton *PBStop;
    QPushButton *PBLeft;
    QPushButton *PBBackWord;
    QGraphicsView *GVController;
    QSplitter *splitter_2;
    QLabel *LSpeed;
    QLabel *label_10;
    QWidget *tab_2;
    QLabel *label_3;
    QListWidget *LWSaveLocations;
    QPushButton *PBAddNewLocation;
    QPushButton *PBEditLocation;
    QFrame *frame;
    QPushButton *PBStartNavigation;
    QFrame *frame_2;
    QLabel *label_8;
    QFrame *rvizContainer;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout;
    QLabel *label_4;
    QLabel *label_5;
    QTextEdit *TEX_coordinate;
    QLabel *label_6;
    QTextEdit *TEY_coordinate;
    QLabel *label_7;
    QTextEdit *TETheta_coordinate;
    QWidget *tab_3;
    QPushButton *PBStartMapping;
    QPushButton *PBPause;
    QPushButton *PBSaveMap;
    QPushButton *PBLoadMap;
    QFrame *frame_3;
    QLabel *label_9;
    QRadioButton *RBmaunal;
    QRadioButton *RB_Automate;
    QWidget *tab_4;
    QPushButton *pbEmergencyStop;
    QProgressBar *PBBattryLevel;
    QLabel *label;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));        
        MainWindow->resize(782, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));      
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(0, 30, 801, 431));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        splitter = new QSplitter(tab);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setGeometry(QRect(50, 330, 311, 41));
        splitter->setOrientation(Qt::Horizontal);
        label_2 = new QLabel(splitter);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    font: bold 14pt \"Segoe UI\";\n"
"    color: #333;\n"
"}\n"
""));
        splitter->addWidget(label_2);
        HSSpeed = new QSlider(splitter);
        HSSpeed->setObjectName(QString::fromUtf8("HSSpeed"));
        HSSpeed->setStyleSheet(QString::fromUtf8("QSlider::groove:horizontal {\n"
"    border: 1px solid #bbb;\n"
"    height: 10px;\n"
"    background: #e0e0e0;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"    background: #00cc44;\n"
"    border: 1px solid #777;\n"
"    height: 10px;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"    background: #ccc;\n"
"    border: 1px solid #777;\n"
"    height: 10px;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"    background: #00aa33;\n"
"    border: 1px solid #5c5c5c;\n"
"    width: 18px;\n"
"    margin: -5px 0;\n"
"    border-radius: 9px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal:hover {\n"
"    background: #009933;\n"
"}\n"
""));
        HSSpeed->setMaximum(100);
        HSSpeed->setSingleStep(10);
        HSSpeed->setValue(10);
        HSSpeed->setSliderPosition(10);
        HSSpeed->setTracking(true);
        HSSpeed->setOrientation(Qt::Horizontal);
        HSSpeed->setInvertedControls(false);
        HSSpeed->setTickPosition(QSlider::TicksAbove);
        HSSpeed->setTickInterval(10);
        splitter->addWidget(HSSpeed);
        layoutWidget = new QWidget(tab);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));        
        layoutWidget->setGeometry(QRect(20, 20, 411, 281));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        PBForward = new QPushButton(layoutWidget);
        PBForward->setObjectName(QString::fromUtf8("PBForward"));
        PBForward->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #4CAF50; /* Default green */\n"
"    color: white;\n"
"    font: bold 13pt \"Segoe UI\";\n"
"    border: none;\n"
"    border-radius: 10px;\n"
"    padding: 10px;\n"
"    min-width: 80px;\n"
"    min-height: 50px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #45a049;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #2e7031;\n"
"}\n"
""));

        gridLayout->addWidget(PBForward, 0, 1, 1, 1);

        PBRight = new QPushButton(layoutWidget);
        PBRight->setObjectName(QString::fromUtf8("PBRight"));
        PBRight->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #FF9800; /* Default green */\n"
"    color: white;\n"
"    font: bold 13pt \"Segoe UI\";\n"
"    border: none;\n"
"    border-radius: 10px;\n"
"    padding: 10px;\n"
"    min-width: 80px;\n"
"    min-height: 50px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #45a049;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #2e7031;\n"
"}\n"
""));

        gridLayout->addWidget(PBRight, 1, 0, 1, 1);

        PBStop = new QPushButton(layoutWidget);
        PBStop->setObjectName(QString::fromUtf8("PBStop"));
        PBStop->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F44336; /* Default green */\n"
"    color: white;\n"
"    font: bold 13pt \"Segoe UI\";\n"
"    border: none;\n"
"    border-radius: 10px;\n"
"    padding: 10px;\n"
"    min-width: 80px;\n"
"    min-height: 50px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #45a049;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #2e7031;\n"
"}\n"
""));

        gridLayout->addWidget(PBStop, 1, 1, 1, 1);

        PBLeft = new QPushButton(layoutWidget);
        PBLeft->setObjectName(QString::fromUtf8("PBLeft"));
        PBLeft->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color:  #FF9800; /* Default green */\n"
"    color: white;\n"
"    font: bold 13pt \"Segoe UI\";\n"
"    border: none;\n"
"    border-radius: 10px;\n"
"    padding: 10px;\n"
"    min-width: 80px;\n"
"    min-height: 50px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #45a049;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #2e7031;\n"
"}\n"
""));

        gridLayout->addWidget(PBLeft, 1, 2, 1, 1);

        PBBackWord = new QPushButton(layoutWidget);
        PBBackWord->setObjectName(QString::fromUtf8("PBBackWord"));
        PBBackWord->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #2196F3; /* Default green */\n"
"    color: white;\n"
"    font: bold 12pt \"Segoe UI\";\n"
"    border: none;\n"
"    border-radius: 10px;\n"
"    padding: 10px;\n"
"    min-width: 80px;\n"
"    min-height: 50px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #45a049;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #2e7031;\n"
"}\n"
""));

        gridLayout->addWidget(PBBackWord, 2, 1, 1, 1);

        GVController = new QGraphicsView(tab);
        GVController->setObjectName(QString::fromUtf8("GVController"));        
        GVController->setGeometry(QRect(440, 20, 150, 150));
        splitter_2 = new QSplitter(tab);
        splitter_2->setObjectName(QString::fromUtf8("splitter_2"));
        splitter_2->setGeometry(QRect(370, 330, 91, 41));
        splitter_2->setOrientation(Qt::Horizontal);
        LSpeed = new QLabel(splitter_2);
        LSpeed->setObjectName(QString::fromUtf8("LSpeed"));
        LSpeed->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    font: bold 14pt \"Segoe UI\";\n"
"    color: #333;\n"
"}\n"
""));
        splitter_2->addWidget(LSpeed);
        label_10 = new QLabel(splitter_2);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    font: bold 14pt \"Segoe UI\";\n"
"    color: #333;\n"
"}\n"
"\n"
""));
        splitter_2->addWidget(label_10);
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        label_3 = new QLabel(tab_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(40, 0, 201, 51));
        label_3->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    font: bold 11pt \"Segoe UI\";\n"
"    color: #333;\n"
"}"));
        LWSaveLocations = new QListWidget(tab_2);
        LWSaveLocations->setObjectName(QString::fromUtf8("LWSaveLocations"));  
        LWSaveLocations->setGeometry(QRect(40, 50, 256, 192));
        PBAddNewLocation = new QPushButton(tab_2);
        PBAddNewLocation->setObjectName(QString::fromUtf8("PBAddNewLocation"));
        PBAddNewLocation->setGeometry(QRect(40, 250, 124, 41));
        PBAddNewLocation->setStyleSheet(QString::fromUtf8("QPushButton {\n"    
"    background-color: #0078D7;       /* Windows blue */\n"
"    color: white;\n"
"    font: bold 12pt \"Segoe UI\";\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    border-radius: 6px;\n"
"    min-width: 100px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #005fa3;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #004c87;\n"
"}\n"
""));
        PBEditLocation = new QPushButton(tab_2);
        PBEditLocation->setObjectName(QString::fromUtf8("PBEditLocation"));    
        PBEditLocation->setGeometry(QRect(170, 250, 124, 41));
        PBEditLocation->setStyleSheet(QString::fromUtf8("QPushButton {\n"      
"    background-color: #ffc107;       /* Windows blue */\n"
"    color: white;\n"
"    font: bold 12pt \"Segoe UI\";\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    border-radius: 6px;\n"
"    min-width: 100px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #005fa3;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #004c87;\n"
"}\n"
""));
        frame = new QFrame(tab_2);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(30, 10, 281, 290));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        PBStartNavigation = new QPushButton(tab_2);
        PBStartNavigation->setObjectName(QString::fromUtf8("PBStartNavigation"));
        PBStartNavigation->setGeometry(QRect(30, 310, 191, 81));
        PBStartNavigation->setStyleSheet(QString::fromUtf8("QPushButton {\n"   
"    background-color: #28a745;       /* Windows blue */\n"
"    color: white;\n"
"    font: bold 12pt \"Segoe UI\";\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    border-radius: 6px;\n"
"    min-width: 100px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #005fa3;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #004c87;\n"
"}\n"
""));
        frame_2 = new QFrame(tab_2);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setGeometry(QRect(329, 10, 441, 290));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        label_8 = new QLabel(frame_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(10, 0, 201, 31));
        label_8->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    font: bold 11pt \"Segoe UI\";\n"
"    color: #333;\n"
"}"));
        rvizContainer = new QFrame(frame_2);
        rvizContainer->setObjectName(QString::fromUtf8("rvizContainer"));      
        rvizContainer->setGeometry(QRect(10, 30, 421, 251));
        rvizContainer->setFrameShape(QFrame::StyledPanel);
        rvizContainer->setFrameShadow(QFrame::Raised);
        layoutWidget1 = new QWidget(tab_2);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));      
        layoutWidget1->setGeometry(QRect(270, 330, 461, 46));
        horizontalLayout = new QHBoxLayout(layoutWidget1);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(layoutWidget1);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    font: bold 11pt \"Segoe UI\";\n"
"    color: #333;\n"
"}"));

        horizontalLayout->addWidget(label_4);

        label_5 = new QLabel(layoutWidget1);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    font: bold 11pt \"Segoe UI\";\n"
"    color: #333;\n"
"}"));

        horizontalLayout->addWidget(label_5);

        TEX_coordinate = new QTextEdit(layoutWidget1);
        TEX_coordinate->setObjectName(QString::fromUtf8("TEX_coordinate"));    
        TEX_coordinate->setStyleSheet(QString::fromUtf8("QTextEdit {\n"        
"    border: 2px solid #ccc;\n"
"    border-radius: 6px;\n"
"    padding: 6px;\n"
"    font: 11pt \"Segoe UI\";\n"
"    background-color: #ffffff;\n"
"    color: #333;\n"
"    min-height: 28px;\n"
"    max-height: 28px;\n"
"       min-width: 34px;\n"
"    max-width: 34px;\n"
"}\n"
""));

        horizontalLayout->addWidget(TEX_coordinate);

        label_6 = new QLabel(layoutWidget1);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    font: bold 11pt \"Segoe UI\";\n"
"    color: #333;\n"
"}"));

        horizontalLayout->addWidget(label_6);

        TEY_coordinate = new QTextEdit(layoutWidget1);
        TEY_coordinate->setObjectName(QString::fromUtf8("TEY_coordinate"));    
        TEY_coordinate->setStyleSheet(QString::fromUtf8("QTextEdit {\n"        
"    border: 2px solid #ccc;\n"
"    border-radius: 6px;\n"
"    padding: 6px;\n"
"    font: 11pt \"Segoe UI\";\n"
"    background-color: #ffffff;\n"
"    color: #333;\n"
"    min-height: 28px;\n"
"    max-height: 28px;\n"
"       min-width: 34px;\n"
"    max-width: 34px;\n"
"}\n"
""));

        horizontalLayout->addWidget(TEY_coordinate);

        label_7 = new QLabel(layoutWidget1);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    font: bold 11pt \"Segoe UI\";\n"
"    color: #333;\n"
"}"));

        horizontalLayout->addWidget(label_7);

        TETheta_coordinate = new QTextEdit(layoutWidget1);
        TETheta_coordinate->setObjectName(QString::fromUtf8("TETheta_coordinate"));
        TETheta_coordinate->setStyleSheet(QString::fromUtf8("QTextEdit {\n"    
"    border: 2px solid #ccc;\n"
"    border-radius: 6px;\n"
"    padding: 6px;\n"
"    font: 11pt \"Segoe UI\";\n"
"    background-color: #ffffff;\n"
"    color: #333;\n"
"    min-height: 28px;\n"
"    max-height: 28px;\n"
"       min-width: 34px;\n"
"    max-width: 34px;\n"
"}\n"
""));

        horizontalLayout->addWidget(TETheta_coordinate);

        tabWidget->addTab(tab_2, QString());
        frame->raise();
        label_3->raise();
        LWSaveLocations->raise();
        PBAddNewLocation->raise();
        PBEditLocation->raise();
        PBStartNavigation->raise();
        frame_2->raise();
        layoutWidget->raise();
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        PBStartMapping = new QPushButton(tab_3);
        PBStartMapping->setObjectName(QString::fromUtf8("PBStartMapping"));    
        PBStartMapping->setGeometry(QRect(450, 50, 151, 61));
        PBStartMapping->setStyleSheet(QString::fromUtf8("QPushButton {\n"      
"    background-color: #0078D7;       /* Windows blue */\n"
"    color: white;\n"
"    font: bold 12pt \"Segoe UI\";\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    border-radius: 6px;\n"
"    min-width: 100px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #005fa3;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #004c87;\n"
"}\n"
""));
        PBPause = new QPushButton(tab_3);
        PBPause->setObjectName(QString::fromUtf8("PBPause"));
        PBPause->setGeometry(QRect(630, 50, 151, 61));
        PBPause->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #0078D7;       /* Windows blue */\n"
"    color: white;\n"
"    font: bold 12pt \"Segoe UI\";\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    border-radius: 6px;\n"
"    min-width: 100px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #005fa3;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #004c87;\n"
"}\n"
""));
        PBSaveMap = new QPushButton(tab_3);
        PBSaveMap->setObjectName(QString::fromUtf8("PBSaveMap"));
        PBSaveMap->setGeometry(QRect(450, 140, 151, 61));
        PBSaveMap->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #0078D7;       /* Windows blue */\n"
"    color: white;\n"
"    font: bold 12pt \"Segoe UI\";\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    border-radius: 6px;\n"
"    min-width: 100px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #005fa3;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #004c87;\n"
"}\n"
""));
        PBLoadMap = new QPushButton(tab_3);
        PBLoadMap->setObjectName(QString::fromUtf8("PBLoadMap"));
        PBLoadMap->setGeometry(QRect(630, 140, 151, 61));
        PBLoadMap->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #0078D7;       /* Windows blue */\n"
"    color: white;\n"
"    font: bold 12pt \"Segoe UI\";\n"
"    padding: 6px 12px;\n"
"    border: none;\n"
"    border-radius: 6px;\n"
"    min-width: 100px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #005fa3;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #004c87;\n"
"}\n"
""));
        frame_3 = new QFrame(tab_3);
        frame_3->setObjectName(QString::fromUtf8("frame_3"));
        frame_3->setGeometry(QRect(40, 30, 381, 331));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        label_9 = new QLabel(tab_3);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(450, 230, 151, 41));
        label_9->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    font: bold 11pt \"Segoe UI\";\n"
"    color: #333;\n"
"}\n"
""));
        RBmaunal = new QRadioButton(tab_3);
        RBmaunal->setObjectName(QString::fromUtf8("RBmaunal"));
        RBmaunal->setGeometry(QRect(450, 270, 99, 21));
        RB_Automate = new QRadioButton(tab_3);
        RB_Automate->setObjectName(QString::fromUtf8("RB_Automate"));
        RB_Automate->setGeometry(QRect(540, 270, 99, 21));
        tabWidget->addTab(tab_3, QString());
        frame_3->raise();
        PBStartMapping->raise();
        PBPause->raise();
        PBSaveMap->raise();
        PBLoadMap->raise();
        label_9->raise();
        RBmaunal->raise();
        RB_Automate->raise();
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        tabWidget->addTab(tab_4, QString());
        pbEmergencyStop = new QPushButton(centralwidget);
        pbEmergencyStop->setObjectName(QString::fromUtf8("pbEmergencyStop"));  
        pbEmergencyStop->setGeometry(QRect(20, 470, 171, 81));
        pbEmergencyStop->setStyleSheet(QString::fromUtf8("QPushButton {\n"     
"    background-color: red;\n"
"    color: yellow;\n"
"    font-size: 12pt;\n"
"    font-weight: bold;\n"
"    border: none;\n"
"    border-radius: 40px;\n"
"    padding: 0px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: darkred;\n"
"}\n"
""));
        PBBattryLevel = new QProgressBar(centralwidget);
        PBBattryLevel->setObjectName(QString::fromUtf8("PBBattryLevel"));      
        PBBattryLevel->setGeometry(QRect(570, 490, 211, 61));
        PBBattryLevel->setStyleSheet(QString::fromUtf8("QProgressBar {\n"      
"    border: 2px solid #555;\n"
"    border-radius: 10px;\n"
"    text-align: center;\n"
"    font: bold 14px;\n"
"    color: black;\n"
"    background-color: #eee;\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background-color: #00cc44;  /* Green color */\n"
"    border-radius: 10px;\n"
"    margin: 1px;\n"
"}\n"
""));
        PBBattryLevel->setValue(60);
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 0, 111, 31));
        label->setTextFormat(Qt::PlainText);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 782, 20));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Alphabot GUI", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Speed", nullptr));
        PBForward->setText(QCoreApplication::translate("MainWindow", "Forward", nullptr));
        PBRight->setText(QCoreApplication::translate("MainWindow", "Right", nullptr));
        PBStop->setText(QCoreApplication::translate("MainWindow", "Stop", nullptr));
        PBLeft->setText(QCoreApplication::translate("MainWindow", "Left", nullptr));
        PBBackWord->setText(QCoreApplication::translate("MainWindow", "BackWord", nullptr));
        LSpeed->setText(QCoreApplication::translate("MainWindow", "0.1", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "m/s", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("MainWindow", "Manual Control", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Saved Locations", nullptr));
        PBAddNewLocation->setText(QCoreApplication::translate("MainWindow", "Add", nullptr));
        PBEditLocation->setText(QCoreApplication::translate("MainWindow", "Edit", nullptr));
        PBStartNavigation->setText(QCoreApplication::translate("MainWindow", "Start Navigation", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "Map Preview", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "Or enter coordicates: ", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "X:", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "Y:", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "\316\230:", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "Go To Location", nullptr));
        PBStartMapping->setText(QCoreApplication::translate("MainWindow", "Start Mapping", nullptr));
        PBPause->setText(QCoreApplication::translate("MainWindow", "Pause", nullptr));
        PBSaveMap->setText(QCoreApplication::translate("MainWindow", "Save Map", nullptr));
        PBLoadMap->setText(QCoreApplication::translate("MainWindow", "Load Map", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "Exploration Mode", nullptr));
        RBmaunal->setText(QCoreApplication::translate("MainWindow", "Manual", nullptr));
        RB_Automate->setText(QCoreApplication::translate("MainWindow", "Automate", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("MainWindow", "Build Map", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QCoreApplication::translate("MainWindow", "Configuration", nullptr));
        pbEmergencyStop->setText(QCoreApplication::translate("MainWindow", "Emergency Stop", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "ROBOT STATUS", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ALPHABOT_GUI_H