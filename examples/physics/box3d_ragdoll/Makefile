# This is a minimal makefile only used for the examples. If you want a makefile
# for your project, take one from the templates inside examples/templates.

BINDIRS	:= data

# Box3D is a separate archive. libNEA.a is Nitro Engine Advanced *without* it,
# so a project asks for physics by name. This line must come before the include
# below, because Makefile.example sets LIBS with ?= and yields to a value that
# is already set.
LIBS	:= -lNEA_box3d -lNEA -lnds9 -lc

include ../../Makefile.example
