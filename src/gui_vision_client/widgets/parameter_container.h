/**
 * \file	parameter_container.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

#include <QScrollArea>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QSpacerItem>
#include "gui_vision_client/widgets/container_widget.h"
#include "gui_vision_client/widgets/parameter.h"

namespace gui_vision_client {

class ParameterContainer : public QScrollArea,
                           public ContainerWidget<Parameter> {
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
  explicit ParameterContainer(QWidget *const parent = nullptr);

  /** Destructor. */
  virtual ~ParameterContainer();

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  /**
   * Sets a layout.
   * \param [in,out]	layout	If non-null, the layout.
   */
  inline void setLayout(QVBoxLayout *layout);

  //==========================================================================
  // P U B L I C   M E T H O D S

  /** Reload spacer. */
  inline void reloadSpacer();

  /** Removes all. */
  virtual void removeAll() override;

 public slots:
  //==========================================================================
  // Q T   S L O T S

  /**
   * Handles the parameter value changed described by parameter1.
   * \param	parameter1	The first parameter.
   */
  void onParameterValueChanged(Parameter *const &);

signals:
  //==========================================================================
  // Q T  S I G N A L S

  /**
   * Parameter value changed.
   * \param	parameter1	The first parameter.
   */
  void parameterValueChanged(Parameter *const &);

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * Creates the widgets.
   * \param	parameter	The parameter.
   */
  virtual void createWidgets(const Parameter *const &parameter) override;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  /** The current parameter. */
  Parameter *_current_parameter;

  /** The layout. */
  QVBoxLayout *_layout;

  /** The spacer. */
  QSpacerItem *_spacer;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void ParameterContainer::setLayout(QVBoxLayout *layout) {
  _layout = layout;
}

//------------------------------------------------------------------------------
//
inline void ParameterContainer::reloadSpacer() {
  _layout->removeItem(_spacer);
  _layout->addItem(_spacer);
}

}  // namespace gui_vision_client
