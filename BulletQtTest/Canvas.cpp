#include "Canvas.h"
#include <QPainter>
#include <iostream>
#include <QFileInfoList>
#include <QDir>
#include <QMessageBox>
#include <QResizeEvent>
#include <QtWidgets/QApplication>

#ifndef SQR
#define SQR(x)	((x) * (x))
#endif

Canvas::Canvas(QWidget *parent) : QWidget(parent) {
	ctrlPressed = false;
	shiftPressed = false;

	animation_timer = NULL;
}

Canvas::~Canvas() {
}

void Canvas::init() {
	sim.init();
	update();
}

void Canvas::run() {
	if (animation_timer == NULL) {
		animation_timer = new QTimer(this);
		connect(animation_timer, SIGNAL(timeout()), this, SLOT(animation_update()));
		animation_timer->start(10);
	}
}

void Canvas::stop() {
	if (animation_timer != NULL) {
		animation_timer->stop();
		delete animation_timer;
		animation_timer = NULL;
	}
}

void Canvas::animation_update() {
	try {
		sim.stepSimulation(0.01);
	}
	catch (char* ex) {
		//kinematics.stepBackward();
		stop();
		std::cerr << "Animation is stopped by error:" << std::endl;
		std::cerr << ex << std::endl;
	}

	update();
}

void Canvas::paintEvent(QPaintEvent *e) {
	QPainter painter(this);

	sim.draw(painter);
}

void Canvas::mousePressEvent(QMouseEvent* e) {
}

void Canvas::mouseMoveEvent(QMouseEvent* e) {
}

void Canvas::mouseReleaseEvent(QMouseEvent* e) {
}

void Canvas::resizeEvent(QResizeEvent *e) {
}

void Canvas::keyPressEvent(QKeyEvent* e) {
	ctrlPressed = false;
	shiftPressed = false;

	if (e->modifiers() & Qt::ControlModifier) {
		ctrlPressed = true;
	}
	if (e->modifiers() & Qt::ShiftModifier) {
		shiftPressed = true;
	}

	switch (e->key()) {
	case Qt::Key_Escape:
		break;
	case Qt::Key_Space:
		break;
	case Qt::Key_Delete:
		break;
	}

	update();
}

void Canvas::keyReleaseEvent(QKeyEvent* e) {
	switch (e->key()) {
	case Qt::Key_Control:
		ctrlPressed = false;
		break;
	case Qt::Key_Shift:
		shiftPressed = false;
		break;
	default:
		break;
	}
}

