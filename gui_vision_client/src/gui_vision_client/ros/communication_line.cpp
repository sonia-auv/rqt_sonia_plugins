/**
 * \file	communication_line.cpp
 * \author  Thomas Fuhrmann <tomesman@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	27/02/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#include "gui_vision_client/ros/communication_line.h"

namespace gui_vision_client {
//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
CommunicationLine::CommunicationLine(QObject *const parent)
    : QObject(parent), _node_handler(), _current_executions() {
  // service de copie de fc
  addServiceClient<proc_image_processing::copy_filterchain>("copy_fc", "copy_filterchain");

  // service d'obtention d'un paramètre d'un filtre
  addServiceClient<proc_image_processing::get_filterchain_filter_param>(
      "get_filter_param", "get_filterchain_filter_param");

  // service d'obtention des paramètres d'un filtre
  addServiceClient<proc_image_processing::get_filterchain_filter_all_param>(
      "get_all_filter_param", "get_filterchain_filter_all_param");

  // service de set des paramètres d'un filtre
  addServiceClient<proc_image_processing::set_filterchain_filter_param>(
      "set_filter_param", "set_filterchain_filter_param");

  // service d'obtention d'un filtre
  addServiceClient<proc_image_processing::get_filterchain_filter>(
      "get_filter", "get_filterchain_filter");

  // service de management d'un filtre
  addServiceClient<proc_image_processing::manage_filterchain_filter>(
      "manage_filter", "manage_filterchain_filter");

  // service de management d'une FilterChain
  addServiceClient<proc_image_processing::manage_filterchain>("manage_fc",
                                                   "manage_filterchain");

  // service de sauvegarde d'une fc
  addServiceClient<proc_image_processing::save_filterchain>("save_fc", "save_filterchain");

  // service pour set l'ordre d'une fc
  addServiceClient<proc_image_processing::set_filterchain_filter_order>(
      "change_filter_order", "set_filterchain_filter_order");

  // service pour récupérer une liste de données
  addServiceClient<proc_image_processing::get_information_list>("info_list",
                                                     "get_information_list");

  // service pour récupérer la filterchain d'une exécution
  addServiceClient<proc_image_processing::get_filterchain_from_execution>(
      "get_fc_from_exec", "get_filterchain_from_execution");

  // service pour récupérer la filterchain d'une exécution
  addServiceClient<proc_image_processing::get_media_from_execution>(
      "get_media_from_exec", "get_media_from_execution");

  // service pour set l'observeur de filtre
  addServiceClient<proc_image_processing::set_filterchain_filter_observer>(
      "set_filter_observer", "set_filterchain_filter_observer");

  // service pour démarrer ou stopper une exécution
  addServiceClient<proc_image_processing::execute_cmd>("execute_cmd", "execute_cmd");
}

//------------------------------------------------------------------------------
//
CommunicationLine::~CommunicationLine() {
  for (auto &service : _services) {
    service.second.shutdown();
  }
  _current_executions.clear();
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename T>
void CommunicationLine::addServiceClient(const std::string &service,
                                         const std::string &node) {
  _services[service] = _node_handler.serviceClient<T>(NODE_NAME_PREFIX + node);
}

//------------------------------------------------------------------------------
//
void CommunicationLine::changeImageSubscriber(const QString &exec_to_launch,
                                              const QString &exec_to_stop) {
  if (exec_to_stop != "") {
    stopExecutionFeed(exec_to_stop);
  }

  // si l'exécution à lancer n'existe pas, on l'ajoute
  if (_current_executions.find(exec_to_launch) == _current_executions.end()) {
    _current_executions.insert(std::pair<const QString, ImageSubscriber *>(
        exec_to_launch, new ImageSubscriber(_node_handler)));
  }

  // on connecte l'observer d'image
  connect(_current_executions[exec_to_launch],
          SIGNAL(imgSubsriberReceivedImage(const cv::Mat &,
                                           const ImageSubscriber *)),
          this,
          SLOT(onReceivedImage(const cv::Mat &, const ImageSubscriber *)));

  // on connecte l'observer de result
  connect(_current_executions[exec_to_launch],
          SIGNAL(imgSubscriberReceivedResult(const QString &,
                                             const ImageSubscriber *)),
          this,
          SLOT(onReceivedResult(const QString &, const ImageSubscriber *)));

  // on lance l'observeur
  _current_executions[exec_to_launch]->change(NODE_NAME_PREFIX +
                                              exec_to_launch.toStdString());
}

//=============================================================================
//  PUBLIC ROS INTERFACES

//------------------------------------------------------------------------------
//
QVector<QString> CommunicationLine::getFilterChainList() {
  proc_image_processing::get_information_list srv;
  srv.request.cmd = FILTERCHAIN;

  return serviceGetList<proc_image_processing::get_information_list>(&srv, "info_list");
}

//------------------------------------------------------------------------------
//
QVector<QString> CommunicationLine::getFiltersForFilterChain(
    const QString &filter_chain_name, const QString &execution_name) {
  proc_image_processing::get_filterchain_filter srv;
  srv.request.exec_name = execution_name.toStdString();
  srv.request.filterchain = filter_chain_name.toStdString();

  return serviceGetList<proc_image_processing::get_filterchain_filter>(&srv, "get_filter");
}

//------------------------------------------------------------------------------
//
QVector<QString> CommunicationLine::getMediaList() {
  proc_image_processing::get_information_list srv;
  srv.request.cmd = MEDIA;

  return serviceGetList<proc_image_processing::get_information_list>(&srv, "info_list");
}

//------------------------------------------------------------------------------
//
QVector<QString> CommunicationLine::getFilterList() {
  proc_image_processing::get_information_list srv;
  srv.request.cmd = FILTER;

  return serviceGetList<proc_image_processing::get_information_list>(&srv, "info_list");
}

//------------------------------------------------------------------------------
//
QVector<QString> CommunicationLine::getExecutionList() {
  proc_image_processing::get_information_list srv;
  srv.request.cmd = EXECUTION;

  return serviceGetList<proc_image_processing::get_information_list>(&srv, "info_list");
}

//------------------------------------------------------------------------------
//
QVector<RawParameter> CommunicationLine::getParametersForFilter(
    const QString &filter_name, const QString &filter_chain_name,
    const QString &execution_name) {
  proc_image_processing::get_filterchain_filter_all_param srv;
  srv.request.exec_name = execution_name.toStdString();
  srv.request.filterchain = filter_chain_name.toStdString();
  srv.request.filter = filter_name.toStdString();

  // on notifie le serveur pour qu'il change d'observeur
  setFilterObserver(execution_name, filter_chain_name, filter_name);

  return toRawParameterList(
      serviceGetList<proc_image_processing::get_filterchain_filter_all_param>(
          &srv, "get_all_filter_param"));
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::setFilterParameter(const QString &filter_chain_name,
                                           const QString &filter_name,
                                           const QString &parameter_name,
                                           const QString &parameter_value,
                                           const QString &execution_name) {
  proc_image_processing::set_filterchain_filter_param srv;
  srv.request.exec_name = execution_name.toStdString();
  srv.request.filterchain = filter_chain_name.toStdString();
  srv.request.filter = filter_name.toStdString();
  srv.request.parameter = parameter_name.toStdString();
  srv.request.value = parameter_value.toStdString();

  return callService<proc_image_processing::set_filterchain_filter_param>(
      &srv, "set_filter_param");
}

//------------------------------------------------------------------------------
//
QString CommunicationLine::getFilterChainFromExecution(
    const QString &execution_name) {
  proc_image_processing::get_filterchain_from_execution srv;
  srv.request.exec_name = execution_name.toStdString();

  return serviceGetString<proc_image_processing::get_filterchain_from_execution>(
      &srv, "get_fc_from_exec");
}

//------------------------------------------------------------------------------
//
QString CommunicationLine::getMediaFromExecution(
    const QString &execution_name) {
  proc_image_processing::get_media_from_execution srv;
  srv.request.exec_name = execution_name.toStdString();

  return serviceGetString<proc_image_processing::get_media_from_execution>(
      &srv, "get_media_from_exec");
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::addFilter(const QString &filter_chain_name,
                                  const QString &filter_name,
                                  const QString &execution_name) {
  return serviceManageFilter(filter_chain_name.toStdString(), ADD,
                             filter_name.toStdString(),
                             execution_name.toStdString());
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::deleteFilter(const QString &filter_chain_name,
                                     const QString &filter_name,
                                     const QString &execution_name) {
  return serviceManageFilter(filter_chain_name.toStdString(), DELETE,
                             filter_name.toStdString(),
                             execution_name.toStdString());
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::createFilterChain(const QString &filter_chain_name) {
  return serviceManageFilterChain(filter_chain_name.toStdString(), ADD);
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::deleteFilterChain(const QString &filter_chain_name) {
  return serviceManageFilterChain(filter_chain_name.toStdString(), DELETE);
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::copyFilterChain(
    const QString &filter_chain_name_to_copy,
    const QString &filter_chain_new_name) {
  proc_image_processing::copy_filterchain srv;
  srv.request.filterchain_to_copy = filter_chain_name_to_copy.toStdString();
  srv.request.filterchain_new_name = filter_chain_new_name.toStdString();
  return callService<proc_image_processing::copy_filterchain>(&srv, "copy_fc");
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::renameFilterChain(
    const QString &filter_chain_old_name,
    const QString &filter_chain_new_name) {
  // first copy
  copyFilterChain(filter_chain_old_name, filter_chain_new_name);
  // then delete the old one
  return serviceManageFilterChain(filter_chain_old_name.toStdString(), DELETE);
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::saveFilterChain(const QString &filter_chain_name,
                                        const QString &execution_name) {
  return serviceSave(filter_chain_name.toStdString(), ADD,
                     execution_name.toStdString());
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::restoreFilterChain(const QString &filter_chain_name,
                                           const QString &execution_name) {
  return serviceSave(filter_chain_name.toStdString(), DELETE,
                     execution_name.toStdString());
}

//------------------------------------------------------------------------------
//
QString CommunicationLine::startExecution(const QString &execution_name,
                                          const QString &filter_chain_name,
                                          const QString &media_name) {
  // création de l'ImagesSubscriber correspondant
  QString exec_name = serviceExecute(execution_name.toStdString(),
                                     filter_chain_name.toStdString(),
                                     media_name.toStdString(), ADD);
  _current_executions.insert(std::pair<const QString, ImageSubscriber *>(
      exec_name, new ImageSubscriber(_node_handler)));
  return exec_name;
}

//------------------------------------------------------------------------------
//
QString CommunicationLine::stopExecution(const QString &execution_name,
                                         const QString &filter_chain_name,
                                         const QString &media_name) {
  stopExecutionFeed(execution_name);
  // on stop l'execution sur le serveur
  return serviceExecute(execution_name.toStdString(),
                        filter_chain_name.toStdString(),
                        media_name.toStdString(), DELETE);
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::stopExecutionFeed(const QString &execution_name) {
  // on supprime l'instance dans le map
  if (_current_executions.find(execution_name) != _current_executions.end()) {
    _current_executions[execution_name]->stop();
    _current_executions.erase(execution_name);
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::setFilterObserver(const QString &execution_name,
                                          const QString &filter_chain_name,
                                          const QString &filter_name) {
  proc_image_processing::set_filterchain_filter_observer srv;
  srv.request.execution = execution_name.toStdString();
  srv.request.filterchain = filter_chain_name.toStdString();
  srv.request.filter = filter_name.toStdString();

  if (!callService<proc_image_processing::set_filterchain_filter_observer>(
          &srv, "set_filter_observer")) {
    return false;
  }

  return srv.response.result;
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::changeFilterOrder(const QString &execution_name,
                                          const QString &filter_chain_name,
                                          const unsigned int &filter_index,
                                          const unsigned int commande) {
  proc_image_processing::set_filterchain_filter_order srv;
  srv.request.exec_name = execution_name.toStdString();
  srv.request.filterchain = filter_chain_name.toStdString();
  srv.request.filter_index = filter_index;
  srv.request.cmd = commande;

  if (!callService<proc_image_processing::set_filterchain_filter_order>(
          &srv, "change_filter_order")) {
    return false;
  }

  return srv.response.success;
}

//=============================================================================
//  MANAGE PARAMETERS

//------------------------------------------------------------------------------
//
QVector<RawParameter> CommunicationLine::toRawParameterList(
    const QVector<QString> &data) const {
  RawParameter parameter;
  QVector<RawParameter> parameters;
  QStringList splitted_data;  //(name|type|value|min|max|description)
  QVector<QString> datas;

  // on parcourt les données d'entrée et on parse chaque élément du vector
  for (auto &it : data) {
    splitted_data = (it).split(QString::fromStdString(PARAMETER_SEPARATOR),
                               QString::KeepEmptyParts);
    datas = QVector<QString>::fromList(splitted_data);

    if (splitted_data.size() != 6) {
      break;
    }

    RawParameter parameter;

    parameter.name = datas.value(0);
    parameter.value = parseParameterType(datas.value(2), datas.value(1));
    parameter.value_min = parseParameterType(datas.value(3), datas.value(1));
    parameter.value_max = parseParameterType(datas.value(4), datas.value(1));
    parameter.description = datas.value(5);
    parameter.min_max_enable = !(datas.value(3).isEmpty());

    parameters.push_back(parameter);
  }

  return parameters;
}

//------------------------------------------------------------------------------
//
QVariant CommunicationLine::parseParameterType(const QString &value,
                                               const QString &type) const {
  if (type == "String") {
    return value;
  } else if (type == "Integer") {
    return value.toInt();
  } else if (type == "Double") {
    return value.toDouble();
  } else if (type == "Boolean") {
    if (value == "0") {
      return false;
    } else {
      return true;
    }
  } else {
    // ROS_INFO( "[VISION CLIENT] Filter parameter type undefined." );
    return false;
  }
}

//==============================================================================
//	CALL ROS SERVICES

//------------------------------------------------------------------------------
//
template <typename T>
bool CommunicationLine::callService(T *const &service,
                                    const std::string &node) {
  bool result = false;
  int counter = 0;

  // This is not really clean but i think it's better to keep all the
  // const on methods parameters and just recreate the string here...
  std::string tmp_node = node;

  // appel au serveur, échec après 3 fail
  do {
    counter++;

    result = _services[tmp_node].call(*service);
    if (result) {
      return true;
    }
  } while (counter < CONNEXION_ATTEMPS);

  // ROS_INFO(( "[VISION CLIENT] " + node +
  //            " : The communication to server has failed." ).c_str());

  return false;
}

//------------------------------------------------------------------------------
//
template <typename T>
QVector<QString> CommunicationLine::serviceGetList(T *const &service,
                                                   const std::string &node) {
  QStringList splitted_data;

  // on appelle le serveur
  QString data = serviceGetString(service, node);

  splitted_data = data.split(QString::fromStdString(LIST_SEPARATOR),
                             QString::SkipEmptyParts);

  return QVector<QString>::fromList(splitted_data);
}

//------------------------------------------------------------------------------
//
template <typename T>
QString CommunicationLine::serviceGetString(T *const &service,
                                            const std::string &node) {
  if (!callService(service, node)) {
    return "";
  }

  std::string data = service->response.list;

  if (data.empty()) {
    // ROS_INFO(( "[VISION CLIENT] " + node +
    //            " : There is no data received from server."
    //            ).c_str());
    return "";
  }

  // ROS_INFO(
  //     ( "[VISION CLIENT] " + node + " : Received datas from server."
  //     ).c_str());

  return QString::fromStdString(data);
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::serviceSave(const std::string &filter_chain,
                                    CMD_MANAGE commande,
                                    const std::string &execution) {
  proc_image_processing::save_filterchain srv;
  srv.request.exec_name = execution;
  srv.request.filterchain = filter_chain;
  srv.request.cmd = commande;

  if (!callService<proc_image_processing::save_filterchain>(&srv, "save_fc")) {
    return false;
  }

  return srv.response.success;
}

//------------------------------------------------------------------------------
//
QString CommunicationLine::serviceExecute(const std::string &execName,
                                          const std::string &filter_chain_name,
                                          const std::string &media_name,
                                          CMD_MANAGE commande) {
  proc_image_processing::execute_cmd srv;
  srv.request.node_name = execName;
  srv.request.filterchain_name = filter_chain_name;
  srv.request.media_name = media_name;
  srv.request.cmd = commande;

  if (!callService<proc_image_processing::execute_cmd>(&srv, "execute_cmd")) {
    return "";
  }

  return QString::fromStdString(srv.response.response);
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::serviceManageFilter(
    const std::string &filter_chain_name, const CMD_MANAGE commande,
    const std::string &filter_name, const std::string &execName) {
  proc_image_processing::manage_filterchain_filter srv;
  srv.request.exec_name = execName;
  srv.request.filterchain = filter_chain_name;
  srv.request.filter = filter_name;
  srv.request.cmd = commande;

  if (!callService<proc_image_processing::manage_filterchain_filter>(&srv,
                                                          "manage_filter")) {
    return false;
  }

  return srv.response.success;
}

//------------------------------------------------------------------------------
//
bool CommunicationLine::serviceManageFilterChain(
    const std::string &filter_chain_name, const CMD_MANAGE commande) {
  proc_image_processing::manage_filterchain srv;
  srv.request.filterchain = filter_chain_name;
  srv.request.cmd = commande;

  if (!callService<proc_image_processing::manage_filterchain>(&srv, "manage_fc")) {
    return false;
  }

  return srv.response.success;
}

//==============================================================================
// Q T   S L O T S

//------------------------------------------------------------------------------
//
void CommunicationLine::onReceivedImage(
    const cv::Mat &image, const ImageSubscriber *subscriber) const {
  // on cherche l'exécution correspondante au subscriber
  for (auto &it : _current_executions) {
    if (it.second == subscriber) {
      emit commLineReceivedImage(image, it.first);
      return;
    }
  }
  // ROS_INFO( "[COMMUNICATION LINE] : no subscriber found in current
  //            executions list, this is not an expected behavior." );
}

//------------------------------------------------------------------------------
//
void CommunicationLine::onReceivedResult(
    const QString &message, const ImageSubscriber *subscriber) const {
  // on cherche l'exécution correspondante au subscriber
  for (auto &it : _current_executions) {
    if (it.second == subscriber) {
      emit commLineReceivedResult(message, it.first);
      return;
    }
  }
  // ROS_INFO( "[COMMUNICATION LINE] : no subscriber found in current
  //            executions list, this is not an expected behavior." );
}

}  // namespace gui_vision_client
