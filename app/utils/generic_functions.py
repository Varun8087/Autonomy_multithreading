# File containing generic functions

def iterate_nested_dict(dictionary, prefix=""):
    key_list = []
    for key, value in dictionary.items():
        if isinstance(value, dict):
            for nest_key in value.keys():
                key_list.append("{}/{}".format(key, nest_key))
        else:
            key_list.append(key)
    return key_list
            

def get_value_from_nested_key(dictionary, nested_keys, separator='/'):
    keys = nested_keys.split(separator)
    value = dictionary
    try:
        for k in keys:
            value = value[k]
        return value
    except (KeyError, TypeError):
        return None
