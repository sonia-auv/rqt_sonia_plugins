/**
 * \file	filter.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

//==============================================================================
// I N C L U D E   F I L E S

// Others librairies and .h
#include <QListWidgetItem>

//==============================================================================
// N A M E S P A C E S   D E C L A R A T I O N S

/**
 * The namespace containing the whole code of this ROS package,
 * not polluating the global namespace is always a good practice and it became
 * a norme at SONIA. Please define your class in specific namespace.
 */
namespace vision_client {
class Filter;
}

//==============================================================================
// C L A S S E S

/**
 * A filter.
 * Multiligne.
 *
 */
class vision_client::Filter : public QObject {
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

  /** The widget. */
  QListWidgetItem *const _widget;

  /** The index. */
  unsigned int _index;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline const QString vision_client::Filter::getName() const { return _name; }

//------------------------------------------------------------------------------
//
inline QListWidgetItem *const vision_client::Filter::getWidget() const {
  return _widget;
}

//------------------------------------------------------------------------------
//
inline const unsigned int vision_client::Filter::getIndex() const {
  return _index;
}

//------------------------------------------------------------------------------
//
inline void vision_client::Filter::setIndex(const unsigned int index) {
  _index = index;
}