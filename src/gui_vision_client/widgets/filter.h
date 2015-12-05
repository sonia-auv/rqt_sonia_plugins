/**
 * \file	filter.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

#include <QListWidgetItem>

namespace gui_vision_client {

class Filter : public QObject {
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
   * \param	name 	The name.
   * \param	index	Zero-based index of the.
   */
  explicit Filter(const QString &name, unsigned int index = 0,
                  QObject *const parent = nullptr);

  /** Destructor. */
  ~Filter();

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  /**
   * Gets the name.
   * \return	The name.
   */
  inline const QString getName() const;

  /**
   * Gets the widget.
   * \return	The widget.
   */
  inline QListWidgetItem *const getWidget() const;

  /**
   * Gets the index.
   * \return	The index.
   */
  inline const unsigned int getIndex() const;

  /**
   * Sets an index.
   * \param	index	Zero-based index of the.
   */
  inline void setIndex(const unsigned int index);

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  /** The name. */
  const QString _name;

  /** The index. */
  unsigned int _index;

  /** The widget. */
  QListWidgetItem *const _widget;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline const QString Filter::getName() const { return _name; }

//------------------------------------------------------------------------------
//
inline QListWidgetItem *const Filter::getWidget() const { return _widget; }

//------------------------------------------------------------------------------
//
inline const unsigned int Filter::getIndex() const { return _index; }

//------------------------------------------------------------------------------
//
inline void Filter::setIndex(const unsigned int index) { _index = index; }

}  // namespace gui_vision_client
