cdef extern from "scope.h":
   ctypedef struct ScopeWindow:
      void (*add)(double *values)
      void (*clear)()
      void (*setBlackBG)(int)

cdef class Scope:
   """A window for collecting and displaying N sequences of data
      
      >>> scope = Scope(N, x, y, width, height [,title] )
      
      creates a window with N sequences. Initially,
      the sliders have no labels, and range from 0 to 1."""
   
   cdef ScopeWindow *win
   cdef int numS
   cdef double *tmp
      
   def __init__(self, int n, int x, int y, int w, int h, char *label=NULL):
      cdef WinCreation wc      
      
      wc.type = SimGUI_Scope
      wc.n = n
      wc.x = x
      wc.y = y
      wc.w = w
      wc.h = h
      wc.title = label
      
      createWindow(&wc)
      
      self.win = <ScopeWindow *>wc.object
      self.numS = n
      self.tmp = <double *> malloc(self.numS*sizeof(double))
      
   def __dealloc__(self):
      closeWindow(<Fl_Window *>self.win)
      free(self.tmp)
      
   def add(self, values):
      """add(value_list)
         
         This function adds the passed values to 
         
         sets all sliders to the values specified in
         the argument. If n=len(value_list) is smaller than the number
         of sliders, only the first n slider positions are modified."""

      if len(values)!=self.numS:
         raise TypeError('You need to pass a list of values, matching this Scope.')
      for i in range(0,self.numS): self.tmp[i] = values[i] # copy to c array
             
      lockGUI()
      self.win.add(self.tmp)
      unlockGUI()
      
   def clear(self):
      lockGUI()
      self.win.clear()
      unlockGUI()

   def setBlackBG(self, int value = 1):
      lockGUI()
      if value:
         self.win.setBlackBG(1)
      else:
         self.win.setBlackBG(0)
      unlockGUI()         
