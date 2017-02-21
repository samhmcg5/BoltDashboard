#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    initUI();
    displayImage();
}

MainWindow::~MainWindow()
{

}

void MainWindow::displayImage()
{
    QPixmap * mypix = new QPixmap("/home/sam/Documents/arrow.png");
    QLabel * pixLabel = new QLabel;
    m_vBox->addWidget(pixLabel);
    pixLabel->setPixmap(*mypix);

    QMatrix rm;
    rm.rotate(-20);
    *mypix = mypix->transformed(rm);
    pixLabel->setPixmap(*mypix);
}

void MainWindow::initUI()
{
    m_Central = new QWidget;
    setCentralWidget(m_Central);

    m_vBox = new QVBoxLayout;
    m_Central->setLayout(m_vBox);
}
