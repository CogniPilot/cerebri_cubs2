#ifndef EFMI_WRAPPER_H_
#define EFMI_WRAPPER_H_

#define EFMI_CAT_(a, b) a##b
#define EFMI_CAT(a, b) EFMI_CAT_(a, b)
#define EFMI_CAT3_(a, b, c) a##b##c
#define EFMI_CAT3(a, b, c) EFMI_CAT3_(a, b, c)

#define EFMI_STATE_TYPE(model) EFMI_CAT(model, State)
#define EFMI_STATE(model, name) EFMI_STATE_TYPE(model) name
#define EFMI_METHOD(model, method) EFMI_CAT3(model, _, method)

#define EFMI_INIT(model, state_ptr)                                                               \
	do {                                                                                     \
		*(state_ptr) = (EFMI_STATE_TYPE(model)){0};                                      \
		EFMI_METHOD(model, startup)(state_ptr);                                          \
		EFMI_METHOD(model, recalibrate)(state_ptr);                                      \
	} while (0)

#define EFMI_RECALIBRATE(model, state_ptr) EFMI_METHOD(model, recalibrate)(state_ptr)
#define EFMI_STEP(model, state_ptr) EFMI_METHOD(model, dostep)(state_ptr)

#endif
