/**
 * \file	main_window_controller.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	27/02/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

//==============================================================================
// I N C L U D E   F I L E S

// C and C++ libraries
#include <ctime>

// Others librairies and .h
#include <QMainWindow>
#include <QListWidgetItem>
#include <QLabel>
#include <QPalette>
#include "ros/ros.h"

// Project's .h
#include "ui_main_window.h"
#include "focused_frame_controller.h"
#include "create_execution_window_controller.h"
#include "manage_filter_chains_window_controller.h"
#include "execution.h"
#include "filter.h"
#include "parameter.h"
#include "communication_line.h"

//==============================================================================
// N A M E S P A C E S   D E C L A R A T I O N S

/**
 * The namespace containing the whole code of this ROS package,
 * not polluating the global namespace is always a good practice and it became
 * a norme at SONIA. Please define your class in specific namespace.
 */
namespace vision_client {
class MainWindowController;
}

/**
 * The namespace Ui is generated by Qt-Designer.
 * It will contain all the classes that manage GUI
 */
namespace Ui {
class MainWindow;
}

//==============================================================================
// C L A S S E S

/**
 * Comment.
 * Multiligne.
 */
class vision_client::MainWindowController : public QMainWindow {
  /**
   * The Q_OBJECT constant provided by Qt.
   * Allow the class to behave as a Widget (provides SLOTS, SIGNALS, etc.)
   */
  Q_OBJECT

 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /**
   * Create an instance of MainWindow
   *
   * \param parent Qt parent Widgets
   */
  explicit MainWindowController(QWidget *const parent = nullptr);

  /** Destructor */
  ~MainWindowController();

  /**
   * Gets current execution.
   *
   * \return	null if it fails, else the current execution.
   */
  inline Execution *getCurrentExecution() const;

  /**
   * Sets current execution.
   *
   * \param [in,out]	execution	If non-null, the execution.
   */
  inline void setCurrentExecution(Execution *execution);

  /**
   * Get the associated FocusedFrameController for a given Execution
   *
   * \param execution The execution to associate to the frame
   * \return The FocusedFrameController associated to the execution
   */
  inline FocusedFrameController *getExecutionFrame(const Execution *const &e);

  inline FocusedFrameController *getCurrentFrame() const;

  inline void setCurrentFrame(FocusedFrameController *const &frame);

 public slots:
  //==========================================================================
  // P U B L I C   S L O T S

  /**
   * Handle the signal @ExecutionContainer#ExecutionChanged() from
   * @ExecutionContainer
   *
   * This will call the @CommunicationLine for changing the ROS subscriber
   * and get the list of the filters for the filter chain associated to
   *display
   * it on the FilterContainer
   *
   * \param	execution	The execution.
   */
  void onExecutionChanged(Execution *const &execution);

  /**
   * Handle the signal @FilterContainer#FilterChanged from @FilterContainer
   *
   * This will call the @CommunicationLine to get the list of the parameters
   * for the selected filter
   *
   * \param	filter  The filter.
   */
  void onFilterChanged(Filter *const &filter);

  /**
   * Handle the signal @ParameterContainer#ParameterChanged from
   *@ParameterContainer
   *
   * This will call the @CommunicationLine in order to alert the server that a
   * changement occurenced on a specific filter
   *
   * \param	parameter The parameter.
   */
  void onParameterChanged(Parameter *const &parameter);

  /**
   * Handle the signal @_ui#actionCreateExecution from @_ui
   *
   * Open the Window widget for adding a new execution to the server
   * It will also provide the requiered informatins such as media list and
   * execution list.
   */
  void onCreateExecutionWindow();

  /**
   * Handle the signal @_ui#actionStopExecution from @_ui
   *
   * Open the Window widget for deleting an existing execution to the server
   */
  void onStopExecutionWindow();

  /**
   * Handle the signal @_execution_window#accepted() from @_execution_window
   *
   * Tell the @CommunicationLine to create a new execution on the server
   * then reload all the executions
   */
  void onCreateExecutionAccepted();

  /**
   * TODO Comment this please
   * Handle the signal ? from ?
   *
   * Do something
   */
  void onManageFilterChainsWindow();

  /**
   * Handle the signal @_ui#actionSaveFilterChain from @_ui
   *
   * Simply call the @CommunicationLine for telling the server to save the
   *current
   * filter chain
   */
  void onSaveFilterChain();

  /**
   * Handle the signal @FocusedFrameController#clicked from
   *@FocusedFrameController
   *
   * Check if the current video frame is the clicked on. If not, change it and
   * display a border around the new one
   *
   * \param	frame	The frame.
   */
  void onVideoFrameClicked(FocusedFrameController *const &frame);

  /**
   * Handle the signal @_ui#action_reload_executions from @_ui
   *
   * If the menu is clicked, it reloads all the executions.
   */
  void onReloadExecutions();

  /**
   * Handles the image changed.
   *
   * \param	image	 	The image.
   * \param	execution	The execution.
   */
  void onImageChanged(const cv::Mat &, const QString &);

  /**
   * Handles the result changed.
   *
   * \param	result   	The result.
   * \param	execution	The execution.
   */
  void onResultChanged(const QString &, const QString &);

  void onFilterGoUp();

  void onFilterGoDown();

  /** Handles the add filter clicked. */
  void onAddFilterClicked();

  /** Handles the remove filter clicked. */
  void onRemoveFilterClicked();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Factories all the execution
   *
   * Calling the @CommunicationLine and tell the @ExecutionContainer to add
   *the
   * components
   */
  void loadExecutions();

  /**
   * Factories all the filters
   *
   * Calling the @CommunicationLine and tell the @FilterContainer to add the
   * components
   */
  void loadFilters();

  /**
   * Factories all the parameters
   *
   * Calling the @CommunicationLine and tell the @ParameterContainer to add
   *the
   * components
   */
  void loadParameters();

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /** The user interface. */
  Ui::MainWindow _ui;

  /** The execution window. */
  CreateExecutionWindowController *_execution_window;

  /** The manage filter chains window. */
  ManageFilterChainsWindowController *_manage_filter_chains_window;

  /** The communication. */
  CommunicationLine _communication;

  /** The current video frame. */
  FocusedFrameController *_current_video_frame;

  /**
   * An association of the frame and the execution
   *
   * Who do want a modular image frame that could be used by other program,
   * this is why we choosed not to have attributes such as _execution or
   *_topic
   * in ImageSubscriber, but rather link it in this verry controller.
   *
   * The thing is we want to be able to have several executions feeding the
   * client at the same time. But have a special curent execution.
   * This is mainly the difference beetween _current_execution and
   * _current_executions.
   * Also, this map aims to provide an accessor of the execution given a topic
   * name, this is usefull when you received an image from the communication
   * line.
   */
  QMap<FocusedFrameController *, Execution *> _current_executions;

  /** The currently selected Filter */
  Filter *_current_filter;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline vision_client::Execution *
vision_client::MainWindowController::getCurrentExecution() const {
  return _current_executions[_current_video_frame];
}

//------------------------------------------------------------------------------
//
inline void vision_client::MainWindowController::setCurrentExecution(
    Execution *execution) {
  QMap<vision_client::FocusedFrameController *, Execution *>::iterator it =
      _current_executions.find(_current_executions.key(execution));
  if (it != _current_executions.end()) {
    _current_executions.erase(it);
  }
  _current_executions[_current_video_frame] = execution;
}

//------------------------------------------------------------------------------
//
inline vision_client::FocusedFrameController *
vision_client::MainWindowController::getExecutionFrame(
    const Execution *const &e) {
  for (const auto &frame : _current_executions.keys()) {
    if (e == _current_executions.value(frame)) {
      return frame;
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
//
inline vision_client::FocusedFrameController *
vision_client::MainWindowController::getCurrentFrame() const {
  return _current_video_frame;
}

//------------------------------------------------------------------------------
//
inline void vision_client::MainWindowController::setCurrentFrame(
    FocusedFrameController *const &frame) {
  _current_video_frame = frame;
}