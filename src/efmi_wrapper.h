#ifndef CUBS2_EFMI_WRAPPER_H_
#define CUBS2_EFMI_WRAPPER_H_

#define CUBS2_EFMI_CAT_(a, b) a##b
#define CUBS2_EFMI_CAT(a, b) CUBS2_EFMI_CAT_(a, b)
#define CUBS2_EFMI_CAT3_(a, b, c) a##b##c
#define CUBS2_EFMI_CAT3(a, b, c) CUBS2_EFMI_CAT3_(a, b, c)

#define CUBS2_EFMI_STATE_TYPE(model) CUBS2_EFMI_CAT(model, State)
#define CUBS2_EFMI_STATE(model, name) CUBS2_EFMI_STATE_TYPE(model) name
#define CUBS2_EFMI_METHOD(model, method) CUBS2_EFMI_CAT3(model, _, method)

#define CUBS2_EFMI_INIT(model, state_ptr)                                                       \
	do {                                                                                     \
		*(state_ptr) = (CUBS2_EFMI_STATE_TYPE(model)){0};                                \
		CUBS2_EFMI_METHOD(model, startup)(state_ptr);                                    \
		CUBS2_EFMI_METHOD(model, recalibrate)(state_ptr);                                \
	} while (0)

#define CUBS2_EFMI_RECALIBRATE(model, state_ptr) CUBS2_EFMI_METHOD(model, recalibrate)(state_ptr)
#define CUBS2_EFMI_STEP(model, state_ptr) CUBS2_EFMI_METHOD(model, dostep)(state_ptr)

#endif
