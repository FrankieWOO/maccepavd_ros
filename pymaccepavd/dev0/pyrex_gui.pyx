
cdef extern from *:
   ctypedef float c_float "const float"
   ctypedef char  const_char  "const char"

cdef extern from "Fl/Fl_Window.H":
   ctypedef struct Fl_Window:
      pass

cdef extern from "gui.h":

   ctypedef enum SimGUI_WinType:
      SimGUI_Unknown = 0
      SimGUI_Ode
      SimGUI_Fader
      SimGUI_Scope
      SimGUI_Matrix
      SimGUI_Bar
   
   ctypedef struct WinCreation:
      SimGUI_WinType type
      int x
      int y
      int w
      int h
      int m
      int n
      char *title
      int error
      void *object
   
   int createWindow(WinCreation *)
   void closeWindow(Fl_Window *)
   void lockGUI()
   void unlockGUI()
   
cdef extern from "stdlib.h":
   void *malloc(int)
   void *calloc(int,int)
   void free(void *)   
   void *realloc(void *, int)
   
include "sliderbox.pyx"    
include "scope.pyx"
