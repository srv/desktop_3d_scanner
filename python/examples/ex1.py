# -*- coding: utf-8 -*-
"""
Created on Wed Jun  6 12:05:02 2018

@author: miquel
"""

# import suma_resta
# import suma_resta as s
from suma_resta import suma, resta
from deteccio.detector import detecta

def run():
    a = 10
    b = 4
    c = suma(a, b)
    d = resta(a, b)
    return c, d
    
if __name__ == "__main__":
    a1, b1 = run()
    c1 = detecta()

