cdef extern from "sliderbox.h":
   ctypedef struct cxx_SliderBox "SliderBox":
      void (*setup)(int number, double min, double max, char *label)
      void (*setValue)(int number, double value)
      void (*setStep)(int number, double value)
      double (*getValue)(int number)
      int (*checkExit)()
      
cdef extern from "gui.h":      
   int awaitSlider(cxx_SliderBox *slb)
      
def createSliderBox(int n, label="SliderBox"):
   slb = SliderBox(n,0,0,240,n*40+40,label)      
   return slb

cdef class SliderBox:
   """A window containing a user-defined number of sliders.
      
      >>> slb = SliderBox(N, x, y, width, height [,title] )
      
      creates a window with N (horizontal sliders) at position (x,y), 
      of size (width,height). Initially,
      the sliders have no labels, and range from 0 to 1. 
   """
   
   cdef cxx_SliderBox *win
   cdef int numS
      
   def __cinit__(self, int n, int x, int y, int w, int h, char *label=NULL):
      cdef WinCreation wc      
      
      wc.type = SimGUI_Fader
      wc.n = n
      wc.x = x
      wc.y = y
      wc.w = w
      wc.h = h
      wc.title = label
            
      createWindow(&wc)
      
      self.win = <cxx_SliderBox *>wc.object
      self.numS = n
      
   def __dealloc__(self):
      closeWindow(<Fl_Window *>self.win)
      
   def setupSlider(self, int num, float minV, float maxV, char *label=NULL, double step=0.0):
      """setupSlider(index, minimum, maximum, [label])
         
         sets the range of slider[index], and optionally attaches a label."""
      if num<0 | num>=self.numS:
         raise IndexError, "Slider index out of bounds!"

      lockGUI()
      self.win.setup(num, minV, maxV, label)
      self.win.setStep(num, step)
      unlockGUI()
      
   def setValue(self, int num, float value):
      """setValue(index, value)
         
         This function sets slider[index] to the specified value."""
      if num<0 | num>=self.numS:
         raise IndexError, "Slider index out of bounds!"
            
      lockGUI()
      self.win.setValue(num, value)
      unlockGUI()
   
   def setStep(self, int num, double value):
      """setStep(index, value)
         
         This function sets the step size of slider[index] to the specified value."""
      if num<0 | num>=self.numS:
         raise IndexError, "Slider index out of bounds!"
            
      lockGUI()
      self.win.setStep(num, value)
      unlockGUI()
      
   def getValue(self, int num):
      """getValue(index)
         
         This function returns the position of slider[index]."""
      if num<0 | num>=self.numS:
         raise IndexError, "Slider index out of bounds!"
            
      return self.win.getValue(num)
      
   def setValues(self, values):
      """setValues(value_list)
         
         This function sets all sliders to the values specified in
         the argument. If n=len(value_list) is smaller than the number
         of sliders, only the first n slider positions are modified."""

      cdef int nval
      
      nval = len(values)
      
      if nval > self.numS:
         nval = self.numS
      lockGUI()
      for n in range(0,nval):
         self.win.setValue(n,values[n])
      unlockGUI()

   def getValues(self):
      """getValues()
      
         Returns all slider positions as a list of floats."""
      pos = []
      cdef int n
      lockGUI()
      for n from 0<=n<self.numS:
         pos.append(self.win.getValue(n))
      unlockGUI()
      return pos
         
   def await(self):
      return awaitSlider(self.win)
      
   def checkExit(self):
      cdef int state
      lockGUI()
      state = self.win.checkExit()
      unlockGUI()
      return state
      
      
