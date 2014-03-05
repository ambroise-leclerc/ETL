/*
 * utility.h
 *
 * Created: 01/03/2014 23:02:55
 *  Author: Ambroise Leclerc
 */ 


#ifndef UTILITY_H_
#define UTILITY_H_

template <class T> struct remove_reference      {typedef T type;};
template <class T> struct remove_reference<T&>  {typedef T type;};
template <class T> struct remove_reference<T&&> {typedef T type;};

template <class T>
typename std::remove_reference<T>::type&& move(T&& t) noexcept {
  return static_cast<typename std::remove_reference<T>::type&&>(t);
}  
  
#endif /* UTILITY_H_ */