/**
 * @author    Hugo3132
 * @copyright 2-clause BSD license
 */
#pragma once

#include <Esp.h>
#include <exception>
#include <tuple>

namespace esp32 {

/**
 * \brief Class for easy thread creation within C++ classes
 *
 * The usage is quite straight forward. By a call of esp32::Thread::create(...)
 * a new thread is created.
 */
class Thread {
private:
#pragma region Helper types to call passed function in new thread
  /**
   * @brief Helper class for the creation of a thread which is used to copy the
   * information required to the new thread
   */
  template<typename T, typename... FunctionParameters>
  class FunctionCaller {
    /**
     * @brief Helper template to expand the tuple elements to function
     * parameters
     */
    template<int...>
    struct seq {};

    /**
     * @brief Helper template to expand the tuple elements to function
     * parameters
     */
    template<int N, int... S>
    struct gens : gens<N - 1, N - 1, S...> {};

    /**
     * @brief Helper template to expand the tuple elements to function
     * parameters
     */
    template<int... S>
    struct gens<0, S...> {
      typedef seq<S...> type;
    };

  protected:
    /**
     * @brief object of type T which contains the function which should be
     * called
     */
    T* obj;

    /**
     * @brief function pointer to the function which should be called by the new
     * thread
     */
    void (T::*function)(FunctionParameters...);

    /**
     * @brief The parameters which should be forwarded to the function
     */
    std::tuple<FunctionParameters...> functionParameters;

  public:
    /**
     * @brief The task handle. Required to be able to delete the FreeRTOS task
     * before returning. Otherwise a HardFault will occur!
     */
    TaskHandle_t taskHandle;

  public:
    /**
     * @brief Construct a new Function Caller object and stores the parameters
     * to be passed to the function
     *
     * @param obj object of type T which contains the function which should be
     * called
     * @param function function pointer to the function which should be called
     * by the new thread
     * @param functionParameters The parameters which should be forwarded to the
     * function
     */
    FunctionCaller(T* obj, void (T::*function)(FunctionParameters...), FunctionParameters... functionParameters)
      : obj(obj)
      , function(function)
      , functionParameters(std::make_tuple(functionParameters...)) {}

    /**
     * @brief calls the function
     */
    void call() {
      try {
        callFunc(typename gens<sizeof...(FunctionParameters)>::type());
      } catch (const std::exception& ex) {
        log_e("Thread %s ended with unhandled exception: %s", pcTaskGetTaskName(taskHandle), ex.what());
      }
    }

  protected:
    /**
     * @brief Helper function to expand the tuple elements to function parametesr
     */
    template<int... S>
    inline void callFunc(seq<S...>) {
      (obj->*function)(std::get<S>(functionParameters)...);
    }
  };
#pragma endregion

public:
  /**
   * \brief Creates a new thread aka FreeRTOS task and returns the thread handle
   *
   * @tparam T the object type which contains the thread function. Typically
   * this is template parameter is found by the type deduction.
   * @tparam FunctionParameters the types of the function parameters forwarded
   * to the thread function. Typically this is template parameter is found by
   * the type deduction.
   *
   * @param name  A descriptive name for the task. This is mainly used to
   * facilitate debugging. Max length defined by configMAX_TASK_NAME_LEN -
   * default is 16.
   * @param stackDepth The size of the task stack specified as the number of
   * bytes. Note that this differs from vanilla FreeRTOS.
   * @param uxPriority : The priority at which the task should run. Systems that
   * include MPU support can optionally create tasks in a privileged (system)
   * mode by setting bit portPRIVILEGE_BIT of the priority parameter. For
   * example, to create a privileged task at priority 2 the uxPriority parameter
   * should be set to ( 2 | portPRIVILEGE_BIT ). Low priority numbers denote low
   * priority tasks. The idle task has priority zero (tskIDLE_PRIORITY).
   * @param xCoreID If the value is tskNO_AFFINITY, the created task is not
   * pinned to any CPU, and the scheduler can run it on any core available.
   * Values 0 or 1 indicate the index number of the CPU which the task should be
   * pinned to.
   * @param obj The object containing the thread member function
   * @param funct The pointer to the member function
   * @param parameters The parameters which should be passed to the function.
   * @warning Ensure that the passed parameters exist if they are not copied
   * after returning from this function
   *
   * \par Example:
   * @code
   * class Foo {
   *   void Bar1() {
   *     // Do some slow stuff
   *   }
   *   void Bar2(std::string s, int i) {
   *     // Do some slow stuff with s and i
   *   }
   * };
   *
   * // ....
   * Foo f;
   * TaskHandle_t thread1 = esp32::Thread::create(f, &Foo::Bar1);
   * TaskHandle_t thread2 = esp32::Thread::create(f, &Foo::Bar2, "String", 42);
   * @endcode
   */
  template<typename T, typename... FunctionParameters>
  static TaskHandle_t create(const char* name,
                             const uint32_t& stackDepth,
                             const UBaseType_t& uxPriority,
                             const BaseType_t xCoreID,
                             T& obj,
                             void (T::*funct)(FunctionParameters...),
                             FunctionParameters... parameters) noexcept {
    auto caller = new FunctionCaller<T, FunctionParameters...>(&obj, funct, parameters...);

    xTaskCreatePinnedToCore(
      Thread::staticThreadFunction<T, FunctionParameters...>, name, stackDepth, caller, uxPriority, &caller->taskHandle, xCoreID);
    return caller->taskHandle;
  }

  /**
   * \brief Creates a new thread aka FreeRTOS task and returns the thread handle
   *
   * @tparam T the object type which contains the thread function. Typically
   * this is template parameter is found by the type deduction.
   * @tparam FunctionParameters the types of the function parameters forwarded
   * to the thread function. Typically this is template parameter is found by
   * the type deduction.
   *
   * @param name  A descriptive name for the task. This is mainly used to
   * facilitate debugging. Max length defined by configMAX_TASK_NAME_LEN -
   * default is 16.
   * @param stackDepth The size of the task stack specified as the number of
   * bytes. Note that this differs from vanilla FreeRTOS.
   * @param uxPriority : The priority at which the task should run. Systems that
   * include MPU support can optionally create tasks in a privileged (system)
   * mode by setting bit portPRIVILEGE_BIT of the priority parameter. For
   * example, to create a privileged task at priority 2 the uxPriority parameter
   * should be set to ( 2 | portPRIVILEGE_BIT ). Low priority numbers denote low
   * priority tasks. The idle task has priority zero (tskIDLE_PRIORITY).
   * @param obj The object containing the thread member function
   * @param funct The pointer to the member function
   * @param parameters The parameters which should be passed to the function.
   * @warning Ensure that the passed parameters exist if they are not copied
   * after returning from this function
   *
   * \par Example:
   * @code
   * class Foo {
   *   void Bar1() {
   *     // Do some slow stuff
   *   }
   *   void Bar2(std::string s, int i) {
   *     // Do some slow stuff with s and i
   *   }
   * };
   *
   * // ....
   * Foo f;
   * TaskHandle_t thread1 = esp32::Thread::create(f, &Foo::Bar1);
   * TaskHandle_t thread2 = esp32::Thread::create(f, &Foo::Bar2, "String", 42);
   * @endcode
   */
  template<typename T, typename... FunctionParameters>
  static TaskHandle_t create(const char* name,
                             const uint32_t& stackDepth,
                             const UBaseType_t& uxPriority,
                             T& obj,
                             void (T::*funct)(FunctionParameters...),
                             FunctionParameters... parameters) noexcept {
    auto caller = new FunctionCaller<T, FunctionParameters...>(&obj, funct, parameters...);

    xTaskCreatePinnedToCore(
      Thread::staticThreadFunction<T, FunctionParameters...>, name, stackDepth, caller, uxPriority, &caller->taskHandle, tskNO_AFFINITY);
    return caller->taskHandle;
  }

protected:
  /**
   * \brief Static thread function. Forwards the call to the given member
   * function
   */
  template<typename T, typename... FunctionParameters>
  static void staticThreadFunction(void* parameters) {
    auto caller = ((FunctionCaller<T, FunctionParameters...>*)parameters);

    // call thread function
    caller->call();

    // delete FreeRTOS task
    vTaskDelete(((FunctionCaller<T, FunctionParameters...>*)parameters)->taskHandle);

    // cleanup memory
    delete ((FunctionCaller<T, FunctionParameters...>*)parameters);
  }
};
} // namespace esp32