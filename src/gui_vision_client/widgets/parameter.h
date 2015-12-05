/**
 * \file	parameter.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

#include <typeinfo>
#include <QObject>
#include <QVariant>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPlainTextEdit>
#include <QLabel>

namespace gui_vision_client {

class Parameter : public QObject {
  /**
   * The Q_OBJECT constant provided by Qt.
   * Allow the class to behave as a Widget (provides SLOTS, SIGNALS, etc.)
   */
  Q_OBJECT

 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /**
   * Constructor.
   * \param	name	   	The name.
   * \param	value	   	The value.
   * \param	description	The description.
   * \param	min		   	The minimum.
   * \param	max		   	The maximum.
   */
  explicit Parameter(const QString &name, const QVariant &value,
                     const QString &description = "",
                     const QVariant &min = INT_MIN,
                     const QVariant &max = INT_MAX,
                     QObject *const parent = nullptr);

  /** Destructor. */
  virtual ~Parameter();

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  /**
   * Gets the name.
   * \return	The name.
   */
  inline const QString getName() const;

  /**
   * Gets the type.
   * \return	The type.
   */
  inline const QString getType() const;

  /**
   * Gets the value.
   * \return	The value.
   */
  inline QVariant getValue() const;

  /**
   * Gets string value.
   * \return	The string value.
   */
  inline QString getStringValue() const;

  /**
   * Sets a value.
   * \param	value	The value.
   */
  void setValue(const QVariant value);

  /**
   * Gets the label.
   * \return	null if it fails, else the label.
   */
  inline QLabel *getLabel() const;

  /**
   * Gets value widget.
   * \return	null if it fails, else the value widget.
   */
  inline QWidget *getValueWidget() const;

  /**
   * Gets main widget.
   * \return	null if it fails, else the main widget.
   */
  inline QWidget *getMainWidget() const;

 public slots:
  //==========================================================================
  // P U B L I C   S L O T S

  /** Change value. */
  void changeValue();

signals:
  //==========================================================================
  // P U B L I C   S I G N A L S

  /**
   * Value changed.
   * \param	parameter1	The first parameter.
   */
  void valueChanged(Parameter *const &) const;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /** Sets up the no description user interface. */
  void setupNoDescriptionUi();

  /** Sets up the description user interface. */
  void setupDescriptionUi();

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /** The name. */
  const QString _name;

  /** The value. */
  QVariant _value;

  /** The type. */
  QString _type;

  /** The description. */
  QString _description;

  /** The name label. */
  QLabel *_name_label;

  /** The main widget. */
  QWidget *_main_widget;

  /** The second vertical layout. */
  QVBoxLayout *_verticalLayout_2;

  /** The body layout. */
  QHBoxLayout *_body_layout;

  /** The description text. */
  QPlainTextEdit *_description_text;

  /** The value layout. */
  QHBoxLayout *_value_layout;

  /** The value widget. */
  QWidget *_value_widget;

  /** The second value widget. */
  QSpinBox *_value_widget2;

  /** The second value widget. */
  QSpinBox *_value_widget_2;

  /** The separator. */
  QFrame *_separator;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline const QString Parameter::getName() const { return _name; }

//------------------------------------------------------------------------------
//
inline const QString Parameter::getType() const { return _type; }

//------------------------------------------------------------------------------
//
inline QVariant Parameter::getValue() const { return _value; }

//------------------------------------------------------------------------------
//
inline QString Parameter::getStringValue() const {
  QString value;

  if (_value.type() == QVariant::Bool) {
    if (_value.toBool()) {
      value = "true";
    } else {
      value = "false";
    }
  } else {
    value = _value.toString();
  }

  return value;
}

//------------------------------------------------------------------------------
//
inline QLabel *Parameter::getLabel() const { return _name_label; }

//------------------------------------------------------------------------------
//
inline QWidget *Parameter::getValueWidget() const { return _value_widget; }

//------------------------------------------------------------------------------
//
inline QWidget *Parameter::getMainWidget() const { return _main_widget; }

}  // namespace gui_vision_client
