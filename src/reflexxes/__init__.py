try:
    # Prefer Type IV
    from .rml_type_iv import *
except ImportError:
    from .rml_type_ii import *
