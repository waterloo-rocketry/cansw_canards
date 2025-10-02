When adding a new type of data log message, all of the following should be updated:
- src/application/logger/log.h
  - Add a new enum value to `log_data_type_t`:
    ```diff
     typedef enum {
    +    LOG_TYPE_XXX = M(unique_small_integer),
     } log_data_type_t;
    ```
  - Add a struct definition of your message's data fields to `log_data_container_t`:
    ```diff
     typedef union __attribute__((packed)) {
    +    struct __attribute__((packed)) {
    +        uint32_t l;
    +        float f;
    +        // ...
    +    } typename;
     } log_data_container_t;
    ```
- scripts/logparse.py
  - Add a new format spec to the `FORMATS` dict:
    ```diff
     FORMATS = {
    +    M(unique_small_integer): Spec(name, format, [field, ...]),
     }
    ```
