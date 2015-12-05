/**
 * \file	vision_server_communication_line.h
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	27/02/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

#include <QObject>
#include <QMap>
#include <QVector>
#include <QString>
#include <QVariant>
#include <QStringList>
#include "ros/ros.h"
#include "image_subscriber.h"
#include <vision_server/vision_server_execute_cmd.h>
#include <vision_server/vision_server_copy_filterchain.h>
#include <vision_server/vision_server_manage_filterchain.h>
#include <vision_server/vision_server_get_filterchain_filter_param.h>
#include <vision_server/vision_server_set_filterchain_filter_param.h>
#include <vision_server/vision_server_get_filterchain_filter_all_param.h>
#include <vision_server/vision_server_get_filterchain_filter.h>
#include <vision_server/vision_server_manage_filterchain_filter.h>
#include <vision_server/vision_server_save_filterchain.h>
#include <vision_server/vision_server_set_filterchain_filter_order.h>
#include <vision_server/vision_server_set_filterchain_filter_observer.h>
#include <vision_server/vision_server_get_information_list.h>
#include <vision_server/vision_server_get_filterchain_from_execution.h>
#include <vision_server/vision_server_get_media_from_execution.h>

namespace gui_vision_client {

//==============================================================================
// G L O B A L   V A R I A B L E S   A N D   S T R U C T

/**
 * The maximal number of try to call a service from the VisionServer.
 * When is number is out of range, the connexion fails and an error is logged.
 */
const int CONNEXION_ATTEMPS = 1;

/**
 * The prefix of the node name. This is a SONIA convention, the node of the
 * VisionServer is called vision_server.
 */
const std::string NODE_NAME_PREFIX = "/vision_server/";

/**
 * The data received from the VisionServer are not parsed.
 * The main parser character is a ";".
 */
const std::string LIST_SEPARATOR = ";";

/**
 * When there are some hierarchical level in the data received from the
 * VisionServer, the second level is "|".
 */
const std::string PARAMETER_SEPARATOR = "|";

/**
 * The RawParameter structure store all the parameters given to desribe a
 * filter.
 * The structure is filled by the toRawParameterList method and is based on the
 * PARAMETER_SEPARATOR.
 */
struct RawParameter {
  /** true if the value has a min and a max value. */
  bool min_max_enable;
  /** The name of the parameter. */
  QString name;
  /**
   * The value of the parameter. The QVariant type enables to change in
   * runtime
   * the type of the variable.
   */
  QVariant value;
  /** The minimum value. */
  QVariant value_min;
  /** The maximum value. */
  QVariant value_max;
  /** The description of the parameter. */
  QString description;
};

/**
 * Handles all the communication with the VisionServer through ROS services.
 *
 * The CommunicationLine is the main interface between the GUI of the
 * VisionClient and the VisionServer.
 * The GUI calls the CommunicationLine in order to send a request on a service
 * from the VisionServer.
 * Then the CommunicationLine parse the data received and convert them
 * in QString format.
 * The CommunicationLine manage the ImageSubscriber objects. There is one object
 * per execution which is running.
 * The Qt signals and slots are used to forward the information
 * to the MainWindowController.
 * All the requests send to the VisionServer are made by the callService.
 * Because the call of a ROS service can fail,
 * the CommunicationLine tries CONNEXION_ATTEMPS times to contact the service.
 */
class CommunicationLine : public QObject {
  /**
   * The Q_OBJECT constant provided by Qt.
   * Allow the class to behave as a Widget (provides SLOTS, SIGNALS, etc.)
   */
  Q_OBJECT

 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  /**
   * Values that represent the different values of the
   * vision_server_get_information_list service.
   */
  enum CMD_GET_INFO_LIST {
    ///< An enum constant representing the execution option
    EXECUTION = 1,
    ///< An enum constant representing the media option
    MEDIA = 2,
    ///< An enum constant representing the filterchain option
    FILTERCHAIN = 3,
    ///< An enum constant representing the filter option
    FILTER = 4
  };

  /**
   * Enum for the services of management,
   * enables to select between add or delete.
   */
  enum CMD_MANAGE {
    ///< An enum constant representing the add option
    ADD = 1,
    ///< An enum constant representing the delete option
    DELETE = 2

  };

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /**
   * Default constructor.
   *
   * Initialize all the service available on the VisionServer.
   */
  explicit CommunicationLine(QObject *const parent = nullptr);

  /** Destructor. */
  virtual ~CommunicationLine();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Change image subscriber.
   *
   * Called by the GUI when the execution running on an ExecutionFrame as
   * changed.
   * This method create the ImageSubscriber object if it does not exist and
   * connect the Qt signals in order to achieve
   * the communication between the ImageSubscriber and the
   * MainWindowController
   * through the CommunicationLine.
   *
   * \param	exec_to_launch	The name of the execution to launch.
   * \param	exec_to_stop  	The name of the execution to stop if one is
   * already
   *                        running on the frame.
   */
  void changeImageSubscriber(const QString &exec_to_launch,
                             const QString &exec_to_stop = nullptr);

  /**
   * Gets filterchains' list.
   *
   * Send a request to the VisionServer in order to get the list of
   *filerchains.
   * Call the vision_server_get_information_list service.
   *
   * \return	A QVector containing filterchains in the received order.
   */
  QVector<QString> getFilterChainList();

  /**
   * Gets filter's list
   *
   * Send a request to the VisionServer in order to get the list of filters.
   * Call the vision_server_get_information_list service.
   *
   * \return	A QVector containing filters in the received order.
   */
  QVector<QString> getFilterList();

  /**
   * Gets the filters contained in a filterchain.
   *
   * Send a request to the VisionServer in order to get the list of the
   *filters
   * contained in a filterchain or an execution.
   * By default it provides the filters of the filterchain which is used by
   *the
   * running execution.
   * If the execution does not exist (or the parameter is empty), it provides
   * the filters of the filterchain.
   * Call the vision_server_get_filterchain_filter service.
   *
   * \param	filter_chain_name Name of the filterchain.
   * \param	execution_name Name of the execution. Let empty to get the
   *                       filters' list associated with a filterchain.
   * \return A QVector containing the filters in the order of processing
   *          in the filterchain.
   */
  QVector<QString> getFiltersForFilterChain(const QString &filter_chain_name,
                                            const QString &execution_name = "");

  /**
   * Gets medias' list.
   *
   * Send a request to the VisionServer in order to get the list of medias.
   * Call the vision_server_get_information_list service.
   *
   * \return	A QVector containing medias in the received order.
   */
  QVector<QString> getMediaList();

  /**
   * Gets executions' list.
   *
   * Send a request to the VisionServer in order to get the list of running
   * executions.
   * Call the vision_server_get_information_list service.
   *
   * \return	A QVector containing executions in the received order.
   */
  QVector<QString> getExecutionList();

  /**
   * Gets the parameters for a filter.
   *
   * Send a request to the VisionServer in order to get the list of the
   * parameters of a filter.
   * The filter has to be contained by a filterchain running on an execution.
   * So it gets the parameters of un running filterchain, it's not impossible
   * that it's not the same as the filterchain
   * stored.
   * Call the vision_server_get_filterchain_filter_param service.
   *
   * \param	filter_name		 	Name of the filter contained in
   *the
   *filterchain.
   * \param	filter_chain_name	Name of the filterchain used by the
   * execution.
   * \param	execution_name   	Name of the execution which is running.
   * \return	A QVector containing the list of filters in the order they
   * appear
   *          in the filterchain.
   */
  QVector<RawParameter> getParametersForFilter(const QString &filter_name,
                                               const QString &filter_chain_name,
                                               const QString &execution_name);

  /**
   * Set the value of a parameter of a filter contained in a filterchain used
   * by a running execution.
   *
   * This method is called when the user change the value of a parameter of a
   *filter.
   * Send a request to the VisionServer in order to set the value of the
   *parameter.
   * Call the vision_server_set_filterchain_filter_param service.
   *
   * \param	filter_chain_name Name of the filterchain which contain the
   *filter.
   * \param	filter_name Name of the filter which contain the parameter.
   * \param	parameter_name Name of the parameter which the value has to be
   *set.
   * \param	parameter_value The value of the parameter.
   * \param	execution_name Name of the running execution using the
   *filterchain.
   * \return	true if it succeeds, false if it fails.
   */
  bool setFilterParameter(const QString &filter_chain_name,
                          const QString &filter_name,
                          const QString &parameter_name,
                          const QString &parameter_value,
                          const QString &execution_name);

  /**
   * Get the filterchain used by the running execution given as parameter.
   *
   * Send a request to the VisionServer in order to get the filterchain of the
   * execution.
   * Call the vision_server_get_filterchain_from_execution service.
   *
   * \param	execution_name	Name of the running execution.
   * \return	The filterchain used by the running execution.
   */
  QString getFilterChainFromExecution(const QString &execution_name);

  /**
   * Get the media used by the running execution given as parameter.
   *
   * Send a request to the VisionServer in order to get the media of the
   * execution.
   * Call the vision_server_get_media_from_execution service.
   *
   * \param	execution_name	Name of the running execution.
   * \return	The media used by the running execution.
   */
  QString getMediaFromExecution(const QString &execution_name);

  /**
   * Adds a filter in a filterchain used by a running execution.
   *
   * Send a request to the VisionServer in order to add the filter given as
   * parameter to the filterchain used by the execution.
   * If the execution is null, it adds the filter to the static filterchain
   * (not loaded).
   * Call the vision_server_manage_filterchain_filter service.
   *
   * \param	filter_chain_name Name of the filterchain in which the filter
   *will
   *                          be added.
   * \param	filter_name Name of the filter to add.
   * \param	execution_name Name of the execution using the filterchain.
   * \return	true if it succeeds, false if it fails.
   */
  bool addFilter(const QString &filter_chain_name, const QString &filter_name,
                 const QString &execution_name);

  /**
   * Deletes a filter in a filterchain used by a running execution.
   *
   * Send a request to the VisionServer in order to delete the filter given as
   * parameter to the filterchain used by the execution.
   * If the execution is null, it deletes the filter to the static
   * filterchain.
   * A static filterchain is a filterchain which is not loaded.
   * Call the vision_server_manage_filterchain_filter service.
   *
   * \param	filter_chain_name	Name of the filterchain in which the
   *filter
   *                          will be deleted.
   * \param	filter_name		 	Name of the filter to delete.
   * \param	execution_name   	Name of the execution using the
   *filterchain.
   * \param	execution_name   	Name of the execution using the
   *filterchain.
   * \return	true if it succeeds, false if it fails.
   */
  bool deleteFilter(const QString &filter_chain_name,
                    const QString &filter_name, const QString &execution_name);

  /**
   * Creates a new filterchain.
   *
   * Send a request to the VisionServer in order to create a filterchain.
   * Call the vision_server_manage_filterchain service.
   *
   * \param	filter_chain_name	Name of the filterchain to create.
   * \return	true if it succeeds, false if it fails.
   */
  bool createFilterChain(const QString &filter_chain_name);

  /**
   * Deletes the filterchain described by filter_chain_name.
   *
   * Send a request to the VisionServer in order to delete a filterchain.
   * If the filterchain is used by a running execution, the delete will fail.
   * Call the vision_server_manage_filterchain service.
   *
   * \param	filter_chain_name	Name of the filterchain to delete.
   * \return	true if it succeeds, false if it fails.
   */
  bool deleteFilterChain(const QString &filter_chain_name);

  /**
   * Sets the observer to the filter given as parameter.
   *
   * As the processing of a filterchain is like a pipe, it is possible to
   * observe the render of a specific filter and
   * all the filters before. This method set this "cursor".
   * The filter has to be used by a filterchain used by a running execution.
   * Send a request to the VisionServer in order to change the filter
   *observer.
   * Call the vision_server_set_filterchain_filter_observer service.
   *
   * \param	execution_name Name of the running execution.
   * \param	filter_chain_name Name of the filterchain used by the execution.
   * \param	filter_name Name of the filter used as the "cursor" of the
   *observer.
   * \return	true if it succeeds, false if it fails.
   */
  bool setFilterObserver(const QString &execution_name,
                         const QString &filter_chain_name,
                         const QString &filter_name);

  /**
   * Copies a filterchain which is not used by a running execution.
   *
   * Send a request to the VisionServer in order to copy the filterchain.
   * Call the vision_server_copy_filterchain service.
   *
   * \param	filter_chain_name_to_copy	The name of the filterchain to
   *copy.
   * \param	filter_chain_new_name	 	The name of the new filterchain
   *(the copy).
   * \return	true if it succeeds, false if it fails.
   */
  bool copyFilterChain(const QString &filter_chain_name_to_copy,
                       const QString &filter_chain_new_name);

  /**
   * Rename a filterchain.
   *
   * This method use copyFilterChain and then deleteFilterChain to rename
   * a filterchain.
   * If the filterchain to rename is used by a running execution, it will
   *fail.
   *
   * \param	filter_chain_old_name	Name of the filterchain to rename.
   * \param	filter_chain_new_name	New name of the filterchain.
   * \return	true if it succeeds, false if it fails.
   */
  bool renameFilterChain(const QString &filter_chain_old_name,
                         const QString &filter_chain_new_name);

  /**
   * Saves a filterchain.
   *
   * When a filterchain is used by a running execution, the filters and their
   * parameters values can be modified.
   * The VisionServer doesn't store the new values until the user click to the
   * save button.
   * This will call this method, which send a request to the VisionServer
   * in order to save the filterchain.
   * If the filterchain is not used by a running execution, this call will
   *fail.
   * Call the vision_server_save_filterchain service.
   *
   * \param	filter_chain_name Name of the filterchain to save.
   * \param	execution_name Name of the execution which use the filterchain
   *to
   *                       save.
   * \return	true if it succeeds, false if it fails.
   */
  bool saveFilterChain(const QString &filter_chain_name,
                       const QString &execution_name);

  /**
   * Restore a filterchain.
   *
   * When a filterchain is used by a running execution, the filters and their
   * parameters values can be modified.
   * The VisionServer doesn't store the new values until the user click to the
   * save button.
   * So it is possible to restore the state of the filterchain stored on the
   *file.
   * Warning, this will override all your modifications.
   * Send a request to the VisionServer in order to restore the filterchain.
   * Call the vision_server_save_filterchain service.
   *
   * \param	filter_chain_name	Name of the filterchain to save.
   * \param	execution_name   	Name of the execution which use the
   *filterchain
   *                          to save.
   * \return	true if it succeeds, false if it fails.
   */
  bool restoreFilterChain(const QString &filter_chain_name,
                          const QString &execution_name);

  /**
   * Starts an execution.
   *
   * An execution applies processing (from a filterchain) on a video provided
   * by a media.
   * Send a request to the VisionServer in order to start an execution.
   * Call the vision_server_execute_cmd service.
   *
   * \param	execution_name   	Name of the execution to start.
   * \param	filter_chain_name	Name of the filterchain which will be
   *used
   *                          by the execution.
   * \param	media_name		 	Name of the media.
   * \return	The name of the execution started on the VisionServer.
   */
  QString startExecution(const QString &execution_name,
                         const QString &filter_chain_name,
                         const QString &media_name);

  /**
   * Stops an execution.
   *
   * Send a request to the VisionServer in order to stop an execution.
   * Call the vision_server_execute_cmd service.
   *
   * \param	execution_name   	Name of the execution to stop.
   * \param	filter_chain_name	Name of the filterchain used by the
   *execution.
   * \param	media_name		 	Name of the media used by the
   *execution.
   * \return	The name of the execution stopped on the VisionServer.
   */
  QString stopExecution(const QString &execution_name,
                        const QString &filter_chain_name,
                        const QString &media_name);

  /**
   * Stops the feed of an execution.
   *
   * The feed of an execution is provided by 2 ROS topics, which are read by
   *an
   * ImageSubscriber object.
   * This method will ask to thie object to unsubscribe to the topics
   * and then destroy it.
   *
   * \param	execution_name	Name of the execution.
   * \return	true if it succeeds, false if it fails.
   */
  bool stopExecutionFeed(const QString &execution_name);

  /**
   * Change the order of a filter in a filterchain.
   *
   * Through the GUI, the user can change up or down the order of a filter ine
   *a
   * filterchain used by a running execution.
   * Send a request to the VisionServer in order to change the order of a
   *filter.
   * Call the vision_server_set_filterchain_filter_order service.
   *
   * \param	execution_name   	Name of the execution which use the
   *filterchain.
   * \param	filter_chain_name	Name of the filterchain which contains
   *the
   *                          filter to move.
   * \param	filter_index	 	Zero-based index of the filter.
   * \param	commande		 	1=UP; 2=DOWN. The increment is
   *1.
   * \return	true if it succeeds, false if it fails.
   */
  bool changeFilterOrder(const QString &execution_name,
                         const QString &filter_chain_name,
                         const unsigned int &filter_index,
                         const unsigned int commande);

 public slots:
  //==========================================================================
  // P U B L I C   S L O T S

  /**
   * Qt slot which is called when a imgSubsriberReceivedImage signal is
   *received
   * from an ImageSubscriber object.
   *
   * This slot forwards the image received from an ImageSubscriber to the
   * MainWindowController through the commLineReceivedImage signal.
   *
   * \param	parameter1	  	The image to forward.
   * \param	parameter2	The ImageSubscriber object which emitted the
   *signal.
   */
  void onReceivedImage(const cv::Mat &, const ImageSubscriber *) const;

  /**
   * Qt slot which is called when a imgSubscriberReceivedResult signal is
   * received from an ImageSubscriber object.
   *
   * This slot forwards the result received from an ImageSubscriber to the
   * MainWindowController through the commLineReceivedResult signal.
   *
   * \param	parameter1	  	The result to forward.
   * \param	parameter2	The ImageSubscriber object which emitted the
   *signal.
   */
  void onReceivedResult(const QString &message, const ImageSubscriber *) const;

signals:
  //==========================================================================
  // P U B L I C   S I G N A L S

  /**
   * Qt signal emitted when the onReceivedImage slot is called.
   *
   * This signal advertises the MainWindowController that a new image is
   * available.
   *
   * \param	parameter1	The image received.
   * \param	parameter2	The ImageSubscriber which received the image.
   */
  void commLineReceivedImage(const cv::Mat &, const QString &) const;

  /**
   * Qt signal emitted when the onReceivedResult slot is called.
   *
   * This signal advertises the MainWindowController that a new result is
   * available.
   *
   * \param	parameter1	The string.
   * \param	parameter2	The string.
   */
  void commLineReceivedResult(const QString &, const QString &) const;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Adds a ROS serviceClient to the _services map.
   *
   * In order to use the ROS services of the VisionServer, ROS serviceClient
   *has
   * to be created and saved. The _services map structure saves those
   *handlers.
   *
   * \tparam	T	The type of the service to use.
   * \param	service	The name of the service.
   * \param	node   	The name of the VisionServer node which handles the
   *service.
   */
  template <typename T>
  void addServiceClient(const std::string &service, const std::string &node);

  /**
   * Call a ROS service.
   *
   * This method calls the ROS service provided as parameter.
   * The call is made at most 3 times, in order to prevent from call fails.
   * If the third call doesn't work, an error is logged.
   *
   * \tparam	T	The type of the service to use.
   * \param	service	The name of the service.
   * \param	node   	The name of the VisionServer node which handles the
   *service.
   * \return	true if the call succeeds, false otherwise
   */
  template <typename T>
  bool callService(T *const &service, const std::string &node);

  /**
   * Used when a service returns a string data to split.
   *
   * Some services of the VisionServer returns a single string which contains
   * a lot of information. This method splits this string and return a data
   * structure of the spliited information.
   * In order to split the string, a QStringList is used. The separator to
   *split
   * is defined as the LIST_SEPARATOR constant.
   * The data fromthe VisionServer are provided by the call of the
   * serviceGetString method.
   *
   * \tparam	T	The type of the service to use.
   * \param	service	The name of the service.
   * \param	node   	The name of the VisionServer node which handles the
   *service.
   * \return	A vector of the split strings.
   */
  template <typename T>
  QVector<QString> serviceGetList(T *const &service, const std::string &node);

  /**
   * Used when a service returns a string data.
   *
   * Each service of the VisionServer doesn't return the same type of data.
   * This method must be used when the service returns a string data type.
   * The call to the VisionServer is made through the callService method.
   * This method converts the std::string received in the QString type.
   *
   * \tparam	T	The type of the service to use.
   * \param	service	The name of the service.
   * \param	node   	The name of the VisionServer node which handles the
   *service.
   * \return	The response string of the service.
   */
  template <typename T>
  QString serviceGetString(T *const &service, const std::string &node);

  /**
   * Used for the vision_server_manage_filterchain_filter service.
   *
   * This method is used to call the vision_server_manage_filterchain_filter
   * service, because of the service parameters to fill.
   *
   * \param	filter_chain_name	Name of the filterchain.
   * \param	commande		 	The commande. \sa CMD_MANAGE
   * \param	filter_name		 	Name of the filter.
   * \param	execName		 	Name of the execution.
   * \return	true if it succeeds, false if it fails.
   */
  bool serviceManageFilter(const std::string &filter_chain_name,
                           const CMD_MANAGE commande,
                           const std::string &filter_name,
                           const std::string &execName = "");

  /**
   * Used for the vision_server_manage_filterchain service.
   *
   * This method is used to call the vision_server_manage_filterchain
   * service, because of the service parameters to fill.
   *
   * \param	filter_chain_name	Name of the filterchain.
   * \param	commande		 	The commande. \sa CMD_MANAGE
   * \return	true if it succeeds, false if it fails.
   */
  bool serviceManageFilterChain(const std::string &filter_chain_name,
                                const CMD_MANAGE commande);

  /**
   * Used for the vision_server_save_filterchain service.
   *
   * This method is used to call the vision_server_save_filterchain
   * service, because of the service parameters to fill.
   *
   * \param	filter_chain_name	Name of the filterchain.
   * \param	commande		 	The commande. \sa CMD_MANAGE
   * \param	execName		 	Name of the execution.
   * \return	true if it succeeds, false if it fails.
   */
  bool serviceSave(const std::string &filter_chain_name, CMD_MANAGE commande,
                   const std::string &execName = "");

  /**
   * Used for the vision_server_execute_cmd service.
   *
   * This method is used to call the vision_server_execute_cmd
   * service, because of the service parameters to fill.
   *
   * \param	execName		 	Name of the execution.
   * \param	filter_chain_name	Name of the filterchain.
   * \param	media_name		 	Name of the media.
   * \param	commande		 	The commande. \sa CMD_MANAGE
   *with
   *1
   *=
   *START,
   *2 = STOP
   * \return	The name of the execution which is concerned.
   */
  QString serviceExecute(const std::string &execName,
                         const std::string &filter_chain_name,
                         const std::string &media_name, CMD_MANAGE commande);

  /**
   * Parse the parameter of a data type.
   *
   * All the data received from the VisionServer is in string type. This
   *method
   * is used to set the type of a data. This means for instance if the string
   * is a number, the typeof the value will be int and no more string.
   * The parser accepts only String, Integer, Double and Boolean types.
   * If the type of a data is another one, the method will log an error.
   * The QVariant return value enables to change in runtime the type of a
   *data.
   *
   * \param	value	The value to parse (it has to be a string value)
   * \param	type 	A string describing the data type of the value
   *parameter.
   *              The type must be String, Integer, Double or Boolean).
   * \return	The value in the type given as a parameter.
   */
  QVariant parseParameterType(const QString &value, const QString &type) const;

  /**
   * Converts a data structure of strings into a RawParameter structure list.
   *
   * The data received from the VisionServer about filters and their
   * parameters
   * are parsed and stored in a RawParameter structure. This method splits
   * each element of the data structure given as parameter and store all the
   * data in a RawParameter structure.
   * As the input of the method is a QVector, the output is a QVector. This
   * means if there are many data to split, each element of the input QVector
   * will be splitted and stored in a RawParameter of the output QVector.
   * The order is respected.
   *
   * \param	data	The QVector of strings to convert in RawParameter
   *structure.
   * \return	The QVector of strings converted in RawParameter structure.
   */
  QVector<RawParameter> toRawParameterList(const QVector<QString> &data) const;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * The list of services which can be called.
   *
   * The first parameter is the name of the service. This is a short name of
   *the
   * service and is only used bu the VisionClient.
   * The second parameter is the name of the service provided by the
   *VisionServer.
   */
  std::map<std::string, ros::ServiceClient> _services;

  /** ROS Node handler. */
  ros::NodeHandle _node_handler;

  /**
   * The list of the executions used by the GUI.
   *
   * This list contains only the executions used by the GUI, this means the
   * execution of the VisionServer linked to an ImageSubscriber object and an
   * image and result feed is displayed on the GUI.
   * This structure as a maximal length of 2 because there are at most 2
   * executions displayed by the GUI.
   */
  std::map<QString, ImageSubscriber *> _current_executions;
};

}  // namespace gui_vision_client
