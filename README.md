# CA10_Project

To run nomenclature, update makeindex command as follows:
makeindex %.nlo -s nomencl.ist -o %.nls -t %.nlg

1. run Latex
2. run MakeIndex command
3. run Latex

For nomenclature, set up ide as follows:
https://www.youtube.com/watch?v=Ss1XfsaAnfs

Nomenclature commands:

Acronym:

\nomenclature[A]{\textbf{SMC}}{Sliding Mode Control}

Symbol

\nomenclature[S]{\vec{\omega}}{AngVel}

Terminology

\nomenclature[T]{Nadir}{Whatever}


Exporting .eps images from Matlab:
https://stackoverflow.com/questions/10985946/how-to-export-the-figure-to-color-eps-in-matlab

print -depsc myplot.eps