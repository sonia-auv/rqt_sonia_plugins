/**
 * \file	filter_container.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

//==============================================================================
// I N C L U D E   F I L E S

// Others librairies and .h
#include <QListWidget>
#include <QListWidgetItem>
#include "ros/ros.h"

// Project's .h
#include "container_widget.h"
#include "filter.h"

//==============================================================================
// N A M E S P A C E S   D E C L A R A T I O N S

/**
 * The namespace containing the whole code of this ROS package,
 * not polluating the global namespace is always a good practice and it became
 * a norme at SONIA. Please define your class in specific namespace.
 */
namespace vision_client {
class FilterContainer;
}

//==============================================================================
// C L A S S E S

/**
 * Comment.
 * Multiligne.
 */
class vision_client::FilterContainer : public QListWidget,
                                       public ContainerWidget<Filter> {
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
   *
   * \param [in,out]	parent	(Optional) If non-null, the parent.
   */
  explicit FilterContainer(QWidget *const parent = nullptr);

  /**
   * Destructor
   */
  ~FilterContainer() {}

  //==========================================================================
  // P U B L I C   M E T H O D S

  /** Removes all. */
  virtual void removeAll() override;

 public slots:
  //==========================================================================
  // P U B L I C   S L O T S

  /**
   * Handles the current item changed.
   *
   * \param [in,out]	current 	If non-null, the current.
   * \param [in,out]	previous	If non-null, the previous.
   */
  void onCurrentItemChanged(QListWidgetItem *current,
                            QListWidgetItem *const &previous);

 signals:
  //==========================================================================
  // P U B L I C   S I G N A L S

  /**
     * Focus changed.
     *
     * \param	parameter1	The first parameter.
     */
  void focusChanged(Filter *const &) const;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * Creates the widgets.
   *
   * \param	filter	Specifies the filter.
   */
  virtual inline void createWidgets(const Filter *const &filter) override;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void vision_client::FilterContainer::createWidgets(
    const Filter *const &filter) {
  addItem(filter->getWidget());
}