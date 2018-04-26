#pragma once

#include <pthread.h>

namespace locks {

enum class LockType { NONE, MUTEX, SPIN };

/**
  Thin wrapper around pthread_mutex_t meant to be tied to a data structure.
*/
class Mutex {
public:
  explicit Mutex() { pthread_mutex_init(&_mutex, nullptr); }
  virtual ~Mutex() { pthread_mutex_destroy(&_mutex); }
  Mutex(const Mutex &) = delete;
  Mutex &operator=(const Mutex &) = delete;
  Mutex(Mutex &&) = delete;
  Mutex &operator=(Mutex &&) = delete;

  inline int lock() { return pthread_mutex_lock(&_mutex); }
  inline int trylock() { return pthread_mutex_trylock(&_mutex); }
  inline int unlock() { return pthread_mutex_unlock(&_mutex); }

private:
  pthread_mutex_t _mutex;
};

/**
  Thin wrapper around pthread_spinlock_t meant to be tied to a data structure.
*/
class Spinlock {
public:
  explicit Spinlock() {
    pthread_spin_init(&_spinlock, PTHREAD_PROCESS_PRIVATE);
  }
  virtual ~Spinlock() { pthread_spin_destroy(&_spinlock); }
  Spinlock(const Spinlock &) = delete;
  Spinlock &operator=(const Spinlock &) = delete;
  Spinlock(Spinlock &&) = delete;
  Spinlock &operator=(Spinlock &&) = delete;

  inline int lock() { return pthread_spin_lock(&_spinlock); }
  inline int trylock() { return pthread_spin_trylock(&_spinlock); }
  inline int unlock() { return pthread_spin_unlock(&_spinlock); }

private:
  pthread_spinlock_t _spinlock;
};
}
