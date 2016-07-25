/**
 * \file	container_widget.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

#include <QVector>

namespace gui_vision_client {

/**
 * A base class for all the widget that are going to contain a component
 *
 * For every component (filter, execution, etc.), we want a QWidget that is
 * going to be able to offer method such as delete or add a new component.
 * In this objective, we funished this abstract class that aim to provide an
 * interface for every component container widget.
 *
 * \tparam	T	Generic type parameter.
 */
template <class T>
class ContainerWidget {
 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /** Default constructor. */
  explicit ContainerWidget();

  /** Destructor */
  virtual ~ContainerWidget();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Adds a component to the container widget.
   *
   * Template method that will store the component into the _components
   * and then call the createWidget pure virtual method to create the
   * widget on the UI.
   *
   * \param	component	The component to add.
   */
  inline void add(T *const &component);

  /**
   * Adds all components to the container widget.
   *
   * For each component on the collection, the method @add(T *const
   *&component);
   * will be called in order to add the component into the _components and
   *display
   * the widget on th UI.
   *
   * \param	components	The components.
   */
  inline void add(const QVector<T *const> &components);

  /**
   * Removes the given component.
   *
   * If the component in parameter is contained by _components, it will
   * be remove from it and the associated UI Widget will be destroyed.
   *
   * \param	component	The component.
   */
  inline void remove(T *const &component);

  /**
   * Query if this object contains the given component.
   *
   * \param	component	The T *const &amp; to test for containment.
   * \return	true if the object is in this collection, false if not.
   */
  inline const bool contains(T *const &component);

  /**
   * Gets all the components in the container.
   *
   * \return	all the components in the container.
   */
  inline QVector<T> getAll() const;

  /**
   * Removes the components in the container.
   *
   * Override this method in order to specify the way you want your
   * items to be removed. Depending on the type of the Widget,
   * the method to destroy it are different (clear(), takeAt(), etc.)
   */
  virtual void removeAll() = 0;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * Creates the widgets.
   *
   * This method is called specifically by add(T *const &component);
   * When you added the component to the _components collection, you
   * want it to be displayed on the UI. As the method to process it
   * are different depending on what container Widget you are using, just
   * override this method to define the behavior.
   *
   * \param	component	The component.
   */
  virtual void createWidgets(const T *const &component) = 0;

  //==========================================================================
  // P R O T E C T E D   M E M B E R S

  /**
   * A collection of all the components contained by the container.
   *
   * As a default behavior, every call to a public method will write
   * on this collection to add or remove a component. In that case,
   * every component on the UI are also pointed by an item in _components.
   */
  QVector<T *> _components;

  /**
   * The current component.
   *
   * With some containers, you certainly want to be able to store the
   * currently selected component (such as a Filter in a QListWidget of
   *Filters)
   */
  T *_current;
};

//==============================================================================
// C / D T O R S   S E C T I O N

/// /!\ As this class is a template class, you can't simply put definition in
/// cpp files.
/// Here are the definitions of the class members.

//------------------------------------------------------------------------------
//
template <class T>
ContainerWidget<T>::ContainerWidget() : _components(), _current(nullptr) {}

//------------------------------------------------------------------------------
//
template <class T>
ContainerWidget<T>::~ContainerWidget() {
  if (_current) {
    delete _current;
  }
}

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
template <class T>
inline void ContainerWidget<T>::add(T *const &component) {
  _components.push_back(component);
  createWidgets(component);
}

//------------------------------------------------------------------------------
//
template <class T>
inline void ContainerWidget<T>::add(const QVector<T *const> &components) {
  for (auto const &it : components) {
    add(it);
  }
}

//------------------------------------------------------------------------------
//
template <class T>
inline void ContainerWidget<T>::remove(T *const &component) {
  _components.remove(_components.indexOf(component));
}

//------------------------------------------------------------------------------
//
template <class T>
inline const bool ContainerWidget<T>::contains(T *const &component) {
  return _components.contains(component);
}

//------------------------------------------------------------------------------
//
template <class T>
inline QVector<T> ContainerWidget<T>::getAll() const {
  return _components;
}

}  // namespace gui_vision_client
