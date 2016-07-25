/**
 * \file	parameter.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	13/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#include "gui_vision_client/widgets/parameter.h"

namespace gui_vision_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//-----------------------------------------------------------------------------
//
Parameter::Parameter(const QString &name, const QVariant &value,
                     const QString &description, const QVariant &min,
                     const QVariant &max, QObject *const parent)
    : QObject(parent),
      _name(name),
      _value(value),
      _description(description),
      _name_label(nullptr),
      _main_widget(nullptr),
      _verticalLayout_2(nullptr),
      _body_layout(nullptr),
      _description_text(nullptr),
      _value_layout(nullptr),
      _value_widget(nullptr),
      _value_widget2(nullptr),
      _value_widget_2(nullptr),  // TODO WTF, why to spinbox with same names ?
      _separator(nullptr) {
  if (_description.isEmpty()) {
    setupNoDescriptionUi();
  } else {
    setupDescriptionUi();
  }

  if (value.type() == QVariant::Int) {
    auto widget = new QSlider(_main_widget);
    widget->setOrientation(Qt::Horizontal);
    if (min != INT_MIN && max != INT_MAX) {
      widget->setMinimum(min.toInt());
      widget->setMaximum(max.toInt());
    }
    widget->setValue(_value.toInt());

    _value_widget = widget;
    _value_widget2 = new QSpinBox(_main_widget);
    _value_widget2->setMinimum(min.toInt());
    _value_widget2->setMaximum(max.toInt());
    _value_widget2->setValue(_value.toInt());

    connect(_value_widget, SIGNAL(valueChanged(int)), _value_widget2,
            SLOT(setValue(int)));

    connect(_value_widget2, SIGNAL(valueChanged(int)), _value_widget,
            SLOT(setValue(int)));

    connect(_value_widget, SIGNAL(valueChanged(int)), this,
            SLOT(changeValue()));
  } else if (value.type() == QVariant::Double) {
    auto widget = new QDoubleSpinBox(_main_widget);
    widget->setValue(_value.toDouble());
    if (min != INT_MIN && max != INT_MAX) {
      widget->setMinimum(min.toDouble());
      widget->setMaximum(max.toDouble());
    }
    _value_widget = widget;

    connect(_value_widget, SIGNAL(valueChanged(double)), this,
            SLOT(changeValue()));
  } else if (value.type() == QVariant::Bool) {
    auto widget = new QCheckBox(_main_widget);
    widget->setChecked(_value.toBool());
    _value_widget = widget;

    connect(_value_widget, SIGNAL(stateChanged(int)), this,
            SLOT(changeValue()));
  } else if (value.type() == QVariant::String) {
    auto widget = new QLineEdit(_main_widget);
    widget->setPlaceholderText(value.toString());
    _value_widget = widget;

    connect(_value_widget, SIGNAL(editingFinished()), this,
            SLOT(changeValue()));
  }

  _value_layout->addWidget(_value_widget);

  if (_value_widget2) {
    _value_layout->addWidget(_value_widget2);
  }
}

//-----------------------------------------------------------------------------
//
Parameter::~Parameter() {
  if (_name_label) {
    delete _name_label;
  }
  if (_main_widget) {
    delete _main_widget;
  }
  // These one generate a segfault.
  // if (_verticalLayout_2) delete _verticalLayout_2;
  // if (_body_layout) delete _body_layout;
  // if (_value_layout) delete _value_layout;
  // if (_value_widget) delete _value_widget;
  // if (_value_widget2) delete _value_widget2;
  // if (_description_text) delete _description_text;
  // if (_value_widget_2) delete _value_widget_2;
  // if (_separator) delete _separator;
}

//==============================================================================
// M E T H O D   S E C T I O N

//-----------------------------------------------------------------------------
//
void Parameter::setupDescriptionUi() {
  _main_widget = new QWidget();

  _verticalLayout_2 = new QVBoxLayout(_main_widget);

  _name_label = new QLabel(_main_widget);
  _name_label->setText(_name);
  _verticalLayout_2->addWidget(_name_label);

  _description_text = new QPlainTextEdit(_main_widget);
  _description_text->setPlainText(_description);
  _description_text->setReadOnly(true);
  _description_text->setMaximumSize(
      QSize(300, 69));  // What ? Thats funny isnt it ?

  _verticalLayout_2->addWidget(_description_text);

  _value_layout = new QHBoxLayout();

  _verticalLayout_2->addLayout(_value_layout);
}

//-----------------------------------------------------------------------------
//
void Parameter::setupNoDescriptionUi() {
  _main_widget = new QWidget();

  _body_layout = new QHBoxLayout(_main_widget);

  _name_label = new QLabel(_main_widget);
  _name_label->setText(_name);
  _body_layout->addWidget(_name_label);

  _value_layout = new QHBoxLayout();

  _body_layout->addLayout(_value_layout);
}

//-----------------------------------------------------------------------------
//
void Parameter::setValue(QVariant const value) {
  QSlider *int_widget = dynamic_cast<QSlider *>(_value_widget);
  QDoubleSpinBox *double_widget = dynamic_cast<QDoubleSpinBox *>(_value_widget);
  QCheckBox *bool_widget = dynamic_cast<QCheckBox *>(_value_widget);
  QLineEdit *str_widget = dynamic_cast<QLineEdit *>(_value_widget);

  if (_value.type() == QVariant::Int && int_widget) {
    int_widget->setValue(value.toInt());
  } else if (_value.type() == QVariant::Double && double_widget) {
    double_widget->setValue(value.toDouble());
  } else if (_value.type() == QVariant::Bool && bool_widget) {
    bool_widget->setChecked(value.toBool());
  } else if (_value.type() == QVariant::String && str_widget) {
    str_widget->setText(value.toString());
  }
}

//=============================================================================
// Q T   S L O T S   S E C T I O N

//-----------------------------------------------------------------------------
//
void Parameter::changeValue() {
  QSlider *int_widget = dynamic_cast<QSlider *>(_value_widget);
  QDoubleSpinBox *double_widget = dynamic_cast<QDoubleSpinBox *>(_value_widget);
  QCheckBox *bool_widget = dynamic_cast<QCheckBox *>(_value_widget);
  QLineEdit *str_widget = dynamic_cast<QLineEdit *>(_value_widget);

  if (int_widget) {
    _value = int_widget->value();
  } else if (double_widget) {
    _value = double_widget->value();
  } else if (bool_widget) {
    _value = bool_widget->isChecked();
  } else if (str_widget) {
    _value = str_widget->text();
  } else {
    return;
  }

  emit valueChanged(this);
}

}  // namespace gui_vision_client
