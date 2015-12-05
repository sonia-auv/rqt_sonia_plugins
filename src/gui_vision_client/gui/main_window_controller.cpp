/**
 * \file	main_window_controller.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	27/02/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#include "main_window_controller.h"

namespace gui_vision_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MainWindowController::MainWindowController(QWidget *const parent)
    : QMainWindow(parent),
      _ui(),
      _execution_window(new CreateExecutionWindowController(this)),
      _manage_filter_chains_window(nullptr),
      _communication(),
      _current_video_frame(nullptr),
      _current_executions(),
      _current_filter(nullptr) {
  _ui.setupUi(this);
  _current_video_frame = _ui.video_frame_left;
  _manage_filter_chains_window =
      new ManageFilterChainsWindowController(_communication, this);
  _ui.parameter_container->setLayout(_ui.parameter_layout);

  qRegisterMetaType<cv::Mat>("cv::Mat");

  // When the selected execution changed, call method onExecutionChanged
  connect(_ui.execution_container, SIGNAL(focusChanged(Execution * const &)),
          this, SLOT(onExecutionChanged(Execution * const &)));

  // When the selected filter changed, call method onFilterChanged
  connect(_ui.filter_container, SIGNAL(focusChanged(Filter * const &)), this,
          SLOT(onFilterChanged(Filter * const &)));

  // When the value of the selected parameter changed, call method
  // onParameterValueChanged
  connect(_ui.parameter_container,
          SIGNAL(parameterValueChanged(Parameter * const &)), this,
          SLOT(onParameterChanged(Parameter * const &)));

  // When the server send a new image, call method handleImageChange
  connect(&_communication,
          SIGNAL(commLineReceivedImage(const cv::Mat &, const QString &)), this,
          SLOT(onImageChanged(const cv::Mat &, const QString &)));

  // When the server send a new image, call method handleImageChange
  connect(&_communication,
          SIGNAL(commLineReceivedResult(const QString &, const QString &)),
          this, SLOT(onResultChanged(const QString &, const QString &)));

  // When action Create Execution is called, call method
  // onCreateExecutionWindow
  connect(_ui.action_create_execution, SIGNAL(triggered()), this,
          SLOT(onCreateExecutionWindow()));

  // When action Create Execution is called, call method
  // onCreateExecutionWindow
  connect(_ui.action_manage_filter_chains, SIGNAL(triggered()), this,
          SLOT(onManageFilterChainsWindow()));

  // When action Stop Execution is called, call method onStopExecutionWindow
  connect(_ui.action_stop_execution, SIGNAL(triggered()), this,
          SLOT(onStopExecutionWindow()));

  // When the execution window is validated, call method
  // handleCreateExecutionAccepted
  connect(_execution_window, SIGNAL(accepted()), this,
          SLOT(onCreateExecutionAccepted()));

  // When action Save FilterChain is called, call method saveFilterChain
  connect(_ui.action_save_filter_chain, SIGNAL(triggered()), this,
          SLOT(onSaveFilterChain()));

  // When action Quit is called, close the application
  connect(_ui.action_quit, SIGNAL(triggered()), qApp, SLOT(quit()));

  // When a video frame is clicked, call method handleVideoFrameClicked
  QObject::connect(_ui.video_frame_left,
                   SIGNAL(clicked(FocusedFrameController * const &)), this,
                   SLOT(onVideoFrameClicked(FocusedFrameController * const &)));

  // When a video frame is clicked, call method handleVideoFrameClicked
  QObject::connect(_ui.video_frame_right,
                   SIGNAL(clicked(FocusedFrameController * const &)), this,
                   SLOT(onVideoFrameClicked(FocusedFrameController * const &)));

  // When the reload execution menu is called, call onReloadExecutions
  QObject::connect(_ui.action_reload_executions, SIGNAL(triggered()), this,
                   SLOT(onReloadExecutions()));

  QObject::connect(_ui.filter_arrow_up, SIGNAL(clicked()), this,
                   SLOT(onFilterGoUp()));

  QObject::connect(_ui.filter_arrow_down, SIGNAL(clicked()), this,
                   SLOT(onFilterGoDown()));

  QObject::connect(_ui.filter_button_add, SIGNAL(clicked()), this,
                   SLOT(onAddFilterClicked()));

  QObject::connect(_ui.filter_button_remove, SIGNAL(clicked()), this,
                   SLOT(onRemoveFilterClicked()));

  // Init Components
  loadExecutions();
}

MainWindowController::~MainWindowController() {
  if (_execution_window) {
    delete _execution_window;
  }
  if (_manage_filter_chains_window) {
    delete _manage_filter_chains_window;
  }
  if (_current_video_frame) {
    delete _current_video_frame;
  }
  if (_current_filter) {
    delete _current_filter;
  }
  _current_executions.clear();
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void MainWindowController::loadExecutions() {
  // Stop all the executions which are currently running
  for (const auto &it : _current_executions) {
    _communication.stopExecutionFeed(it->getName());
  }
  _current_executions.clear();

  _ui.execution_container->removeAll();
  _ui.filter_container->removeAll();
  _ui.parameter_container->removeAll();

  const auto executions_names = _communication.getExecutionList();

  if (executions_names.isEmpty()) {
    return;
  }

  // Remove comboBox Items
  _ui.video_frame_left->getComboBox()->clear();
  _ui.video_frame_right->getComboBox()->clear();

  for (const auto &execution_name : executions_names) {
    const auto media_name =
        _communication.getMediaFromExecution(execution_name);
    const auto filter_chain_name =
        _communication.getFilterChainFromExecution(execution_name);

    if (execution_name.isEmpty() || media_name.isEmpty() ||
        filter_chain_name.isEmpty()) {
      continue;
    }

    // Add the executin to the execution table
    _ui.execution_container->add(
        new Execution(execution_name, filter_chain_name, media_name));

    // Add the execution to the left combobox
    _ui.video_frame_left->getComboBox()->addItem(execution_name);

    // Add the execution to the right combobox
    _ui.video_frame_right->getComboBox()->addItem(execution_name);
  }
}

//------------------------------------------------------------------------------
//
void MainWindowController::loadFilters() {
  _ui.filter_container->removeAll();
  _ui.parameter_container->removeAll();
  const auto filters_names = _communication.getFiltersForFilterChain(
      getCurrentExecution()->getFilterChainName(),
      getCurrentExecution()->getName());

  int index = 0;
  for (const auto &filter_name : filters_names) {
    if (filter_name.isEmpty()) {
      continue;
    }

    _ui.filter_container->add(new Filter(filter_name, index));
    index++;
  }
}

//------------------------------------------------------------------------------
//
void MainWindowController::loadParameters() {
  _ui.parameter_container->removeAll();

  // If no execution is currently running on this frame, just do nothing
  if (getCurrentExecution() == nullptr) {
    return;
  }

  const auto parameters = _communication.getParametersForFilter(
      _current_filter->getName(), getCurrentExecution()->getFilterChainName(),
      getCurrentExecution()->getName());

  for (auto &parameter : parameters) {
    if ((parameter).min_max_enable) {
      _ui.parameter_container->add(new Parameter(
          (parameter).name, (parameter).value, (parameter).description,
          (parameter).value_min, (parameter).value_max));
    } else {
      _ui.parameter_container->add(new Parameter(
          (parameter).name, (parameter).value, (parameter).description));
    }
  }

  // Push all widget on top of scroll area
  _ui.parameter_container->reloadSpacer();
}

//==============================================================================
// Q T   S L O T S

//------------------------------------------------------------------------------
//
void MainWindowController::onExecutionChanged(Execution *const &execution) {
  _ui.action_save_filter_chain->setEnabled(true);
  // An other execution is running on the current frame...
  if (_current_executions[_current_video_frame]) {
    _communication.changeImageSubscriber(
        execution->getName(),
        _current_executions[_current_video_frame]->getName());
  } else {
    _communication.changeImageSubscriber(execution->getName());
  }
  setCurrentExecution(execution);
  loadFilters();
}

//------------------------------------------------------------------------------
//
void MainWindowController::onFilterChanged(Filter *const &filter) {
  _current_filter = filter;
  loadParameters();
}

//------------------------------------------------------------------------------
//
void MainWindowController::onParameterChanged(Parameter *const &parameter) {
  _communication.setFilterParameter(
      getCurrentExecution()->getFilterChainName(), _current_filter->getName(),
      parameter->getName(), parameter->getStringValue(),
      getCurrentExecution()->getName());
}

//------------------------------------------------------------------------------
//
void MainWindowController::onCreateExecutionWindow() {
  _execution_window->setFilterChainList(_communication.getFilterChainList());
  _execution_window->setMediaList(_communication.getMediaList());
  _execution_window->show();
}

//------------------------------------------------------------------------------
//
void MainWindowController::onManageFilterChainsWindow() {
  _manage_filter_chains_window->show();
}

//------------------------------------------------------------------------------
//
void MainWindowController::onStopExecutionWindow() {
  if (getCurrentExecution() != nullptr) {
    // deleting the execution
    _communication.stopExecution(getCurrentExecution()->getName(),
                                 getCurrentExecution()->getFilterChainName(),
                                 getCurrentExecution()->getMediaName());

    loadExecutions();
  }
}

//------------------------------------------------------------------------------
//
void MainWindowController::onCreateExecutionAccepted() {
  _communication.startExecution(_execution_window->getExecution(),
                                _execution_window->getFilterChain(),
                                _execution_window->getMedia());

  loadExecutions();
}

//------------------------------------------------------------------------------
//
void MainWindowController::onSaveFilterChain() {
  _communication.saveFilterChain(getCurrentExecution()->getFilterChainName(),
                                 getCurrentExecution()->getName());
}

//------------------------------------------------------------------------------
//
void MainWindowController::onVideoFrameClicked(
    FocusedFrameController *const &frame) {
  if (frame != _current_video_frame) {
    _current_video_frame->unfocus();
    frame->focus();
    _current_video_frame = frame;
  }
}

//------------------------------------------------------------------------------
//
void MainWindowController::onReloadExecutions() { loadExecutions(); }

//------------------------------------------------------------------------------
//
void MainWindowController::onImageChanged(const cv::Mat &image,
                                          const QString &running_execution) {
  for (auto &execution : _current_executions) {
    if (!(execution->getName() == running_execution)) {
      continue;
    }

    getExecutionFrame(execution)->getImageFrame()->changeImage(image);
  }
  // TODO Replace the log by a real logger
  // ROS_INFO( "[MAIN WINDOW CONTROLLER] : I didn't find any frame for this
  //            execution, this is not an expected behavior" );
}

//------------------------------------------------------------------------------
//
void MainWindowController::onResultChanged(const QString &result,
                                           const QString &running_execution) {
  for (auto &execution : _current_executions.keys()) {
    if ((_current_executions.value(execution)->getName() ==
         running_execution)) {
      execution->getResultLine()->setText(result);
      return;
    }
  }
  // TODO Replace the log by a real logger
  // ROS_INFO( "[MAIN WINDOW CONTROLLER] : I didn't find any frame for this
  //            execution, this is not an expected behavior" );
}

//------------------------------------------------------------------------------
//
void MainWindowController::onAddFilterClicked() {
  auto filter_list = QStringList{};
  const auto available_filter = _communication.getFilterList();
  for (const auto &filter : available_filter) {
    filter_list << tr(filter.toStdString().c_str());
  }

  auto ok = false;
  const auto filter =
      QInputDialog::getItem(this, "Add a Filter", "Choose the filter to add",
                            filter_list, 0, false, &ok);
  if (ok) {
    _communication.addFilter(getCurrentExecution()->getFilterChainName(),
                             filter, getCurrentExecution()->getName());
    loadFilters();
  }
  loadFilters();
}

//------------------------------------------------------------------------------
//
void MainWindowController::onRemoveFilterClicked() {
  _communication.deleteFilter(getCurrentExecution()->getFilterChainName(),
                              _current_filter->getName(),
                              getCurrentExecution()->getName());
  loadFilters();
}

//------------------------------------------------------------------------------
//
void MainWindowController::onFilterGoUp() {
  _communication.changeFilterOrder(getCurrentExecution()->getName(),
                                   getCurrentExecution()->getFilterChainName(),
                                   _current_filter->getIndex(), 1);
  loadFilters();
}

//------------------------------------------------------------------------------
//
void MainWindowController::onFilterGoDown() {
  _communication.changeFilterOrder(getCurrentExecution()->getName(),
                                   getCurrentExecution()->getFilterChainName(),
                                   _current_filter->getIndex(), 2);
  loadFilters();
}

}  // namespace gui_vision_client
