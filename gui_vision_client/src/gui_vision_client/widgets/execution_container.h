/**
 * \file	execution_container.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

#include <QTableWidget>
#include <QTableWidgetItem>
#include "gui_vision_client/widgets/container_widget.h"
#include "gui_vision_client/widgets/execution.h"

namespace gui_vision_client {

class ExecutionContainer : public QTableWidget,
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
  virtual ~ExecutionContainer() = default;

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

}  // namespace gui_vision_client
