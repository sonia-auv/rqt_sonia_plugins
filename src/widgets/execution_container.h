/**
 * \file	execution_container.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

//==============================================================================
// I N C L U D E   F I L E S

// Others librairies and .h
#include <QTableWidget>
#include <QTableWidgetItem>

// Project's .h
#include "container_widget.h"
#include "execution.h"

//==============================================================================
// N A M E S P A C E S   D E C L A R A T I O N S

/**
 * The namespace containing the whole code of this ROS package,
 * not polluating the global namespace is always a good practice and it became
 * a norme at SONIA. Please define your class in specific namespace.
 */
namespace vision_client {
class ExecutionContainer;
}

//==============================================================================
// C L A S S E S

/**
 * An execution container.
 * Multiligne.
 *
 */
class vision_client::ExecutionContainer : public QTableWidget,
                                          public ContainerWidget<Execution> {
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
   * \param [in,out]	parent	(Optional) If non-null, the parent.
   */
  explicit ExecutionContainer(QWidget *const parent = nullptr);

  /** Destructor */
  ~ExecutionContainer() {}

  //==========================================================================
  // P U B L I C   M E T H O D S

  /** Removes all. */
  virtual void removeAll() override;

 public slots:
  //============================================================================
  // P U B L I C   S L O T S

  /**
   * Handles the current item changed.
   * \param [in,out]	current 	If non-null, the current.
   * \param [in,out]	previous	If non-null, the previous.
   */
  void onCurrentItemChanged(QTableWidgetItem *current,
                            QTableWidgetItem *previous);

  /**
   * Handles the outside execution changed described by execution_name.
   * \param	execution_name	Name of the execution.
   */
  void onOutsideExecutionChanged(const QString &execution_name);

signals:
  //==========================================================================
  // P U B L I C   S I G N A L S

  /**
   * Focus changed.
   * \param	parameter1	The first parameter.
   */
  void focusChanged(Execution *const &);

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * Creates the widgets.
   * \param	execution	The execution.
   */
  virtual void createWidgets(const Execution *const &execution) override;
};
