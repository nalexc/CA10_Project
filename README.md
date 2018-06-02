# CA10_Project

The TODO margin extension needs to be removed at finalization

## Nomenclature 

To run nomenclature, update makeindex command as follows:

```
makeindex %.nlo -s nomencl.ist -o %.nls -t %.nlg
```

1. run Latex
2. run MakeIndex command
3. run Latex

For nomenclature, set up ide as follows:
https://www.youtube.com/watch?v=Ss1XfsaAnfs

### Nomenclature commands

**Acronym**

```
\nomenclature[A]{\textbf{SMC}}{Sliding Mode Control}
```

**Symbol**

```
\nomenclature[S]{\vec{\omega}}{AngVel}
```

**Terminology**

```
\nomenclature[T]{Nadir}{Whatever}
```

**Forcing alphabetical ordering**

By default, the alphabetical ordering in symbols takes the commands into account. \underline is always before \vec.
to make sure the symbols are ordered the way we expect, we should add the phonetic text after the symbol prefix as follows

```
\nomenclature[Somega]{\vec{\omega}}{AngVel}

\nomenclature[SA]{\underline{\A}}{Transformation matrix}

etc.
```

## Exporting .eps images from Matlab

https://stackoverflow.com/questions/10985946/how-to-export-the-figure-to-color-eps-in-matlab

Save current figure as .eps file:

```
print -depsc myplot.eps
```

## Forcing figures to appear the same place  as in the .tex file

add **[H]** after \begin{figure}

```
\begin{figure}[H]
\centering
\includegraphics{slike/visina8}
\caption{Write some caption here}\label{visina8}
\end{figure}

```

## Export graph

```
subplot(3,1,2)
grid on
legend('wheel 1','wheel 2','wheel 3')
xlabel('time (s)')
ylabel('Torque(Nm) - x axis')
title('Hybrid controller detumbling torque demand')
xlim([0 10])
ylim([-0.4 0.8])
print -depsc detumbling.eps

 plot(control.Data(:,1))
```