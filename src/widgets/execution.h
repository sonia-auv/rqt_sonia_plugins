/**
 * \file	execution.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

//==============================================================================
// I N C L U D E   F I L E S

// Others librairies and .h
#include <QObject>
#include <QString>
#include <QTableWidgetItem>

//==============================================================================
// N A M E S P A C E S   D E C L A R A T I O N S

/**
 * The namespace containing the whole code of this ROS package,
 * not polluating the global namespace is always a good practice and it became
 * a norme at SONIA. Please define your class in specific namespace.
 */
namespace vision_client {
class Execution;
}

//==============================================================================
// C L A S S E S

/**
 * Comment.
 * Multiligne.
 */
class vision_client::Execution : public QObject {
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
   * \param	execution_name   	Name of the execution.
   * \param	filter_chain_name	Name of the filter chain.
   * \param	media_name		 	Name of the media.
   */
  explicit Execution(const QString &execution_name,
                     const QString &filter_chain_name,
                     const QString &media_name);

  /** Destructor. */
  ~Execution();

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  /**
   * Gets the name.
   * \return	The name.
   */
  inline const QString getName() const;

  /**
   * Gets media name.
   * \return	The media name.
   */
  inline const QString getMediaName() const;

  /**
   * Gets filter chain name.
   * \return	The filter chain name.
   */
  inline const QString getFilterChainName() const;

  /**
   * Gets execution widget.
   * \return	The execution widget.
   */
  inline QTableWidgetItem *const getExecutionWidget() const;

  /**
   * Gets filter chain widget.
   * \return	The filter chain widget.
   */
  inline QTableWidgetItem *const getFilterChainWidget() const;

  /**
   * Gets media widget.
   * \return	The media widget.
   */
  inline QTableWidgetItem *const getMediaWidget() const;

  /**
   * Gets is screenshot enable.
   * \return	true if it succeeds, false if it fails.
   */
  inline bool isScreenshotEnabled() const;

  /**
   * Sets is screenshot enable.
   * \param	value	true to value.
   */
  inline void setIsScreenshotEnabled(bool value);

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  /** The name. */
  const QString _name;

  /** Name of the media. */
  const QString _media_name;

  /** Name of the filter chain. */
  const QString _filter_chain_name;

  /** true if this object is screenshot enable. */
  bool _is_screenshot_enabled;

  /** The execution widget. */
  QTableWidgetItem *const _execution_widget;

  /** The filter chain widget. */
  QTableWidgetItem *const _filter_chain_widget;

  /** The media widget. */
  QTableWidgetItem *const _media_widget;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline const QString vision_client::Execution::getName() const { return _name; }

//------------------------------------------------------------------------------
//
inline const QString vision_client::Execution::getMediaName() const {
  return _media_name;
}

//------------------------------------------------------------------------------
//
inline const QString vision_client::Execution::getFilterChainName() const {
  return _filter_chain_name;
}

//------------------------------------------------------------------------------
//
inline QTableWidgetItem *const vision_client::Execution::getExecutionWidget()
    const {
  return _execution_widget;
}

//------------------------------------------------------------------------------
//
inline QTableWidgetItem *const vision_client::Execution::getFilterChainWidget()
    const {
  return _filter_chain_widget;
}

//------------------------------------------------------------------------------
//
inline QTableWidgetItem *const vision_client::Execution::getMediaWidget()
    const {
  return _media_widget;
}

//------------------------------------------------------------------------------
//
inline bool vision_client::Execution::isScreenshotEnabled() const {
  return _is_screenshot_enabled;
}

//------------------------------------------------------------------------------
//
inline void vision_client::Execution::setIsScreenshotEnabled(bool value) {
  _is_screenshot_enabled = value;
}
