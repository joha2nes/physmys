#ifndef LIST_H_
#define LIST_H_

#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define LIST_DEFAULT_CAPACITY 4

#define list_set_capacity(_list, _capacity)\
do {\
    assert((_list)->count < (_capacity));\
    if ((_list)->capacity == (_capacity)) break;\
    void *newArray = malloc((_capacity) * sizeof(*(_list)->items));\
    if ((_list)->count > 0) memcpy(newArray, (_list)->items,  (_list)->count * sizeof(*(_list)->items));\
    free((_list)->items);\
    (_list)->items = newArray;\
    (_list)->capacity = (_capacity);\
} while (0)\

#define list_ensure_capacity(_list, _min)\
do {\
    if ((_list)->capacity < (_min)) {\
        int newCapacity = (_list)->capacity == 0 ? LIST_DEFAULT_CAPACITY : 2 * (_list)->capacity;\
        if (newCapacity < (_min)) newCapacity = (_min);\
        list_set_capacity((_list), newCapacity);\
    }\
} while (0)\

#define list_alloc(_list, _capacity)\
do {\
    (_list)->items = malloc(sizeof(*(_list)->items) * (_capacity));\
    (_list)->capacity = (_capacity);\
} while (0)\

#define list_free(_list)\
do {\
    free((_list)->items);\
    (_list)->capacity = 0;\
} while (0)\

#define list_add(_list, _value)\
do {\
    list_ensure_capacity((_list), (_list)->count + 1);\
    (_list)->items[(_list)->count] = (_value);\
    (_list)->count++;\
} while (0)\

#define list_remove(_list, _value)\
do {\
    for (int _i = 0; _i < (_list)->count; ++_i) {\
        if ((_list)->items[_i] == (_value)) {\
            list_remove_at((_list), _i);\
            break;\
        }\
    }\
} while (0)\

#define list_remove_at(_list, _index)\
do {\
    assert((_index) >= 0 && (_index) < (_list)->count);\
    if ((_index) < (_list)->count - 1) {\
        memmove(&(_list)->items[_index], &(_list)->items[(_index) + 1], \
                ((_list)->count - (_index) - 1) * sizeof(*(_list)->items));\
    }\
    (_list)->count--;\
} while (0)\

#define list_clear(_list) (_list)->count = 0

#define list(_T) struct { _T* items; int count; int capacity; }

#define mk_list(_name, _t, _cap) list(_t) _name = { .items = malloc(sizeof(_t) * _cap), .capacity = _cap, .count = 0 }

#define list_bracket(_name, _t, _cap, _code)\
do\
{\
    mk_list(_name, _t, _cap);\
    _code\
    list_free(&_name);\
}\
while (0)\

#endif // LIST_H_