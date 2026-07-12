% This material is declared a work of the United States Government and is not
% subject to copyright protection in the United States.
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and sell copies
% of the Software, and to permit persons to whom the Software is furnished to do
% so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% ------------------------------------------------------------------------------
% Author: David Woodburn
% Description: class lecture presentation template
% ------------------------------------------------------------------------------

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{antclass}[2025/03/31 ANT Center Class]
\def\@version{20250331}

% Process options.
\newif\ifstandard\standardfalse
\newif\ifdefineul\defineultrue
\DeclareOption{standard}{\standardtrue}
\DeclareOption{wide}{\standardfalse}
\DeclareOption{noul}{\defineulfalse}
\DeclareOption*{\PackageWarning{antbrief}{Unknown '\CurrentOption'}}
\ProcessOptions\relax
\ifstandard
    \LoadClass[10pt]{article}
\else
    \LoadClass[12pt]{article}
\fi

% Packages
\RequirePackage[T1]{fontenc}
\RequirePackage{geometry}
\RequirePackage{fancyhdr}
\RequirePackage{float}
\RequirePackage{flafter}
\RequirePackage[table]{xcolor}
\RequirePackage{pgfplots}
\RequirePackage{adjustbox}
\RequirePackage{sourcesanspro}
\RequirePackage[scaled=0.89]{beramono}
\RequirePackage{amsmath, amsfonts, amssymb}
\RequirePackage{bm}
\RequirePackage{cancel}
\RequirePackage{listings}
\RequirePackage{matlab-prettifier}
\RequirePackage{upquote}
\RequirePackage{array}
\RequirePackage[absolute]{textpos}
\RequirePackage{enumitem}
\RequirePackage{hyperref}

\usepackage{graphicx, animate}
\usepackage[spanish]{babel}
\usepackage{svg}

% TikZ libraries
\usetikzlibrary{math}

% PGF Plot configuration
\pgfplotsset{compat=newest}

% ----------------------
% Added content features
% ----------------------

% Standard math macros
\ifdefineul
    \newcommand{\ul}[1]
        {{}\mkern1mu\underline{\mkern-1mu#1\mkern-1mu}\mkern1mu}
\fi
\DeclareSymbolFont{euler}{U}{eur}{m}{n}
\DeclareMathSymbol{\PI}{\mathord}{euler}{25}

% Convenience macros
\DeclareMathOperator{\sgn}{sgn}
\DeclareMathOperator{\tr}{tr}
\DeclareMathOperator{\adj}{adj}
\DeclareMathOperator*{\argmax}{arg\,max}
\DeclareMathOperator*{\argmin}{arg\,min}
\newcommand{\abs}[1]{\left\lvert#1\right\rvert}
\newcommand{\norm}[1]{\left\lVert#1\right\rVert}
\newcommand{\ssm}[1]{[#1]_{\rule{0pt}{1.38ex}\times}}
\newcommand{\diff}[2]{\frac{\mathrm{d}{#1}}{\mathrm{d}{#2}}}
\newcommand{\piff}[2]{\frac{\partial{#1}}{\partial{#2}}}
\newcommand{\EE}[2]{\ensuremath{{{#1}\!\times\!10^{#2}}}}
\newcommand{\rom}[1]{\uppercase\expandafter{\romannumeral #1\relax}}

% Upright math letters
\newcommand{\upc}[1][]{\mathrm{c}_{#1}}
\newcommand{\ups}[1][]{\mathrm{s}_{#1}}
\newcommand{\upt}[1][]{\mathrm{t}_{#1}}
\newcommand{\upd}{\mathrm{d}}
\newcommand{\upe}{\mathrm{e}}
\newcommand{\upi}{\mathrm{i}}
\newcommand{\upj}{\mathrm{j}}
\newcommand{\upk}{\mathrm{k}}

% Increase maximum number of matrix columns.
\setcounter{MaxMatrixCols}{20}

% Define custom table column types and spacings.
\newcolumntype{L}[1]{>{\raggedright\arraybackslash}p{#1}}
\newcolumntype{C}[1]{>{\centering\arraybackslash}p{#1}}
\newcolumntype{R}[1]{>{\raggedleft\arraybackslash}p{#1}}
\renewcommand{\arraystretch}{1.3}
\setlength{\tabcolsep}{1ex}

% Custom colors
\definecolor{blue}{RGB}{0,0,255}
\definecolor{azure}{RGB}{0,128,255}
\definecolor{cyan}{RGB}{0,209,209}
\definecolor{green}{RGB}{15,180,0}
\definecolor{lime}{RGB}{180,216,0}
\definecolor{yellow}{RGB}{255,191,0}
\definecolor{orange}{RGB}{255,128,0}
\definecolor{red}{RGB}{255,0,0}
\definecolor{magenta}{RGB}{255,0,255}
\definecolor{purple}{RGB}{128,0,255}
\definecolor{colorA}{RGB}{0,38,76}
\definecolor{colorB}{RGB}{4,47,89}
\definecolor{colorC}{RGB}{10,56,102}
\definecolor{colorD}{RGB}{17,66,115}
\definecolor{colorE}{RGB}{25,76,127}
\definecolor{colorH}{RGB}{58,112,166}

% Clickable mailto email addresses
\newcommand{\mailto}[1]{\href{mailto:#1}{\nolinkurl{#1}}}

% Horizontal line with middle text
\newsavebox{\@btext}
\newlength{\@bwidth}
\newcommand{\breaktext}[2][colorH]{%
    \sbox{\@btext}{#2}%
    \setlength{\@bwidth}{0.5\textwidth}%
    \addtolength{\@bwidth}{-0.5\wd\@btext}%
    \addtolength{\@bwidth}{-1ex}%
    \par{\color{#1!20}\rule[0.5ex]{\@bwidth}{1pt}}%
    \hfill{\usebox{\@btext}}\hfill%
    {\color{#1!20}\rule[0.5ex]{\@bwidth}{1pt}}\par}

% Vertical and horizontal centering
\newenvironment{Center}
    {\topskip0pt\par\vspace*{\fill}\vspace*{-\parskip}\vspace*{-\topsep}%
    \begin{minipage}{\textwidth}\centering}
    {\end{minipage}\vspace*{\fill}\vspace*{-\topsep}\par}

% Arbitrary object placement
\newcommand{\pos}[4][]{%
    \if\relax\detokenize{#1}\relax
        \sbox{\@tempboxa}{#4}
        \begin{textblock*}{\wd\@tempboxa}(#2,#3)
            \usebox{\@tempboxa}
        \end{textblock*}
    \else
        \begin{textblock*}{#1}(#2,#3)
            #4
        \end{textblock*}
    \fi}

% Colored text box
\definecolor{shade}{RGB}{213,227,255}
\newcommand{\shadedbox}[2][shade]{\setlength{\fboxsep}{5pt}%
    \hspace*{-5pt}%
    \colorbox{#1}{\parbox{\columnwidth}{#2}}%
    \hspace*{-5pt}}

% -------------------------------------
% Custom code styles (listings package)
% -------------------------------------

% Pseudocode
\lstdefinestyle{pseudostyle}{
    xleftmargin=30pt, xrightmargin=0pt,
    numbers=left, numberstyle=\scriptsize\ttfamily\color{gray}, numbersep=10pt,
    upquote=true, showstringspaces=false, tabsize=4,
    breaklines=true, breakindent=20pt, breakatwhitespace=true,
    breakautoindent=true,
    aboveskip=0.8em, belowskip=0.2em,
    basicstyle=\linespread{1.1}\ttfamily\small,
    commentstyle=\color{gray}\itshape,
    stringstyle=\color{orange},
    columns=flexible}
\lstdefinelanguage{pseudolang}{
    keywords={function, for, while, do, break, continue,
        if, then, else, end, in, to, until, return},
    keywordstyle=\color{black}\bfseries,
    morecomment=[l]{\%\ },
    morestring=[b]",
    morestring=[b]',
    mathescape=true,
    escapechar=`}
\lstnewenvironment{pseudocode}[1][1]{%
    \lstset{language=pseudolang, style=pseudostyle, firstnumber=#1}}{}
\lstnewenvironment{pseudocodetight}[1][1]{%
    \lstset{language=pseudolang, style=pseudostyle, firstnumber=#1,
        xleftmargin=15pt, numbersep=10pt, rulecolor=\color{white}}}{}

% Python code
\lstdefinestyle{pythonstyle}{
    xleftmargin=40pt, xrightmargin=0pt,
    frame=leftline, framesep=9pt, framerule=2pt, rulecolor=\color{azure!20},
    numbers=left, numberstyle=\scriptsize\ttfamily\color{gray}, numbersep=20pt,
    upquote=true, showstringspaces=false, tabsize=4,
    breaklines=true, breakindent=20pt, breakatwhitespace=true,
    breakautoindent=true,
    aboveskip=0.8em, belowskip=0.2em,
    basicstyle=\ttfamily\footnotesize,
    commentstyle=\color{gray},
    stringstyle=\color{orange},
    keywordstyle=[1]\color{azure},
    keywordstyle=[2]\color{azure},
    keywordstyle=[3]\color{purple},
    keywordstyle=[4]\color{magenta}}
\lstdefinelanguage{pythonlang}{
    keywords={and, as, assert, break, class, continue, def, del, elif, else,
        except, False, finally, for, from, global, if, import, in, is, lambda,
        None, nonlocal, not, or, pass, raise, return, True, try, while, with,
        yield},
    morekeywords=[2]{abs, aiter, all, any, anext, ascii, bin, bool, breakpoint,
        bytearray, bytes, callable, chr, classmethod, compile, complex, delattr,
        dict, dir, divmod, enumerate, eval, exec, exit, filter, float, format,
        frozenset, getattr, globals, hasattr, hash, help, hex, id, input, int,
        isinstance, issubclass, iter, len, list, locals, map, max, memoryview,
        min, next, object, oct, open, ord, quit, pow, print, property, range,
        repr, reversed, round, set, setattr, slice, sorted, staticmethod, str,
        sum, super, tuple, type, vars, zip},
    morecomment=[l]{\#\ },
    morecomment=[s]{"""}{"""},
    morestring=[b]",
    morestring=[b]',
    prebreak={ \char92},
    escapechar=`}
\lstnewenvironment{python}[1][1]{%
    \lstset{language=pythonlang, style=pythonstyle, firstnumber=#1}}{}
\newcommand*{\inputpython}[3]{\lstinputlisting[linerange={{#2}-{#3}},
    firstnumber=#2, language=pythonlang, style=pythonstyle]{#1}}
\newcommand*{\py}{\lstinline[language=pythonlang, basicstyle=\ttfamily]}

% MATLAB code
\lstdefinestyle{matlabstyle}{
    style=Matlab-editor,
    basicstyle=\ttfamily\footnotesize,
    xleftmargin=40pt, xrightmargin=0pt,
    frame=leftline, framesep=9pt, framerule=2pt, rulecolor=\color{green!20},
    numbers=left, numberstyle=\scriptsize\ttfamily\color{gray}, numbersep=20pt,
    upquote=true, showstringspaces=false, tabsize=4,
    breaklines=true, breakindent=20pt, breakatwhitespace=true,
    breakautoindent=true,
    aboveskip=0.8em, belowskip=0.2em,
    prebreak={ ...},
    escapechar=`}
\lstnewenvironment{matlab}[1][1]{%
    \lstset{style=matlabstyle, firstnumber=#1}}{}
\newcommand*{\inputmatlab}[3]{\lstinputlisting[
    style=matlabstyle, linerange={{#2}-{#3}}, firstnumber=#2]{#1}}
\newcommand*{\mat}{\lstinline[style=Matlab-editor, basicstyle=\ttfamily]}

% R code
\lstdefinestyle{rstyle}{
    xleftmargin=40pt, xrightmargin=0pt,
    frame=leftline, framesep=9pt, framerule=2pt, rulecolor=\color{purple!20},
    numbers=left, numberstyle=\scriptsize\ttfamily\color{gray}, numbersep=20pt,
    upquote=true, showstringspaces=false, tabsize=4,
    breaklines=true, breakindent=20pt, breakatwhitespace=true,
    breakautoindent=true,
    aboveskip=0.8em, belowskip=0.2em,
    basicstyle=\linespread{1.2}\ttfamily\footnotesize,
    commentstyle=\color{green},
    stringstyle=\color{red},
    keywordstyle=\color{azure}}
\lstdefinelanguage{rlang}{
    language=R,
    morekeywords={TRUE, FALSE},
    deletekeywords={data, frame, length, as, character, gamma, beta},
    prebreak={},
    escapechar=`}
\lstnewenvironment{rlang}[1][1]{%
    \lstset{language=rlang, style=rstyle, firstnumber=#1}%
    \colorlet{ruleColor}{purple}}{}
\newcommand*{\inputrlang}[3]{\lstinputlisting[linerange={{#2}-{#3}},
    firstnumber=#2, language=rlang, style=rstyle]{#1}}
\newcommand*{\rlng}{\lstinline[language=rlang, basicstyle=\ttfamily]}

% Line highlighting
\newcommand{\HL}{\tikz[overlay]{\draw[black, line width=2pt]
    (-10pt,-4pt) -- (-10pt,10pt);}}

% ---------------------------
% General document formatting
% ---------------------------

\renewcommand\familydefault{\sfdefault}

\pgfplotsset{%
    tick label style={font=\scriptsize\sffamily},
    every axis label={font=\sffamily},
    legend style={font=\tiny\sffamily},
    label style={font=\sffamily},
    nodes near coords style=
        {font=\scriptsize,/pgf/number format/assume math mode}
}

% Define margins.
\ifstandard
    \newlength{\@xw}\setlength{\@xw}{6cm}
\else
    \newlength{\@xw}\setlength{\@xw}{8cm}
\fi
\newlength{\@yh}\setlength{\@yh}{4.5cm}
\newlength{\@mgnT}\setlength{\@mgnT}{1.5cm}
\newlength{\@mgnB}\setlength{\@mgnB}{0.499cm}
\newlength{\@mgnL}\setlength{\@mgnL}{1.5cm}
\newlength{\@mgnC}\setlength{\@mgnC}{1cm}
\newlength{\@mgnR}\setlength{\@mgnR}{1cm}
\newlength{\@mgnS}\setlength{\@mgnS}{\@mgnT}
\addtolength{\@mgnS}{-1cm}\addtolength{\@mgnS}{0.348cm}

% Page geometry
\geometry{
    paperwidth=2\@xw, paperheight=2\@yh,
    top=\@mgnT, bottom=\@mgnB,
    left=\@mgnL, right=\@mgnR,
    columnsep=\@mgnC,
    headheight=1cm, headsep=\@mgnS}
\setlength{\columnseprule}{0.1pt}

% Derived dimensions
\newlength{\@xShift}\setlength{\@xShift}{\@xw}
\addtolength{\@xShift}{-\@mgnL}
\newlength{\@yShift}\setlength{\@yShift}{\@mgnT}
\addtolength{\@yShift}{-4.078cm}
\newlength{\@xHead}\setlength{\@xHead}{-0.5\@mgnL}
\addtolength{\@xHead}{0.5\@mgnR}
\newlength{\@yHead}\setlength{\@yHead}{-4cm}

% Paragraph parameters
\setlength{\parindent}{0pt}
\setlength{\parskip}{1.2ex plus 6pt}
\setlength{\emergencystretch}{\hsize}
\overfullrule=1mm
\raggedbottom
\DeclareRobustCommand\justifying{%
    \let\\\@normalcr
    \@rightskip\z@skip \rightskip\@rightskip
    \leftskip\z@skip
    \finalhyphendemerits=5000
    \parfillskip=0pt plus 1fil}

% Reduce list spacing.
\setlist{
    itemindent=0pt,
    labelsep=0.2cm,
    labelwidth=0.4cm,
    leftmargin=0.6cm,
    topsep=0pt,
    parsep=0.5\parskip,
    itemsep=0.5\parskip,
    before=\raggedright}
\setlist[trivlist]{topsep=0pt}

% Hyperlink styling
\hypersetup{
    colorlinks=true,
    linkcolor=colorE,
    citecolor=colorE,
    urlcolor=colorE,
    hypertexnames=false}
\def\UrlFont{\sffamily\itshape}

\pdfsuppresswarningpagegroup=1
\frenchspacing

% Title metadata commands
\newif\ifpretitle\pretitlefalse
\def\@pretitle{}
\newcommand{\pretitle}[1]{\pretitletrue\def\@pretitle{#1}}
\newif\iftitle\titlefalse
\def\@title{}
\renewcommand{\title}[1]{\titletrue\def\@title{#1}}
\newif\ifsubtitle\subtitlefalse
\def\@subtitle{}
\newcommand{\subtitle}[1]{\subtitletrue\def\@subtitle{#1}}
\newif\ifauthor\authorfalse
\def\@author{}
\renewcommand{\author}[1]{\authortrue\def\@author{#1}}

% TikZ predefined drawings
\tikzset{
    antlogocompass/.pic={},
    antlogoant/.pic={},
    antlogocenter/.pic={},
    titlepage/.pic={
        \fill[colorB] (-\@xw,-4.5) rectangle (-\@xw+3cm,4.5);
        \fill[colorC] (-\@xw,3.0) -- ({-\@xw+0.75cm},3.75) --
            ({-\@xw+3cm},1.5) -- ({-\@xw+3cm},-3.0) --
            ({-\@xw+2.25cm},-3.75) -- (-\@xw,-1.5);
        \fill[colorA] (-\@xw,4.5) -- ({-\@xw+1.5cm},4.5) --
            ({-\@xw+0.75cm},3.75) ({-\@xw+3cm},-4.5) --
            ({-\@xw+1.5cm},-4.5) -- ({-\@xw+2.25cm},-3.75);
        \fill[colorD] ({-\@xw+1.5cm},3.75) -- ({-\@xw+2.25cm},4.5) --
            ({-\@xw+3cm},4.5) -- ({-\@xw+3cm},2.25)
            ({-\@xw+1.5cm},-3.75cm) --({-\@xw+0.75cm},-4.5) --
            (-\@xw,-4.5) -- (-\@xw,-2.25cm);
        \begin{scope}[xshift={-\@xw+1.5cm}]
            \node[inner sep=0] at (0,0)
              {\includegraphics[width=2.6cm]{Pics/logos/logospng5.png}};
        \end{scope}
        \newlength{\@titlewidth}
        \setlength{\@titlewidth}{2\@xw}
        \addtolength{\@titlewidth}{-4.5cm}
        \node[right, inner sep=0] at ({3.5cm-\@xw},0)
            {\parbox[c][7cm][c]{\@titlewidth}{%
                \raggedright
                \ifpretitle
                    \normalsize\color{black}{\@pretitle} \\[1.5ex]
                \fi
                \iftitle
                    \Large\color{black}{\bfseries{\@title}} \\[1.5ex]
                \fi
                \ifsubtitle
                    \large\color{colorH}{\bfseries{\@subtitle}}%
                    \\[2ex]
                \fi
                \ifauthor
                    \normalsize\color{black}{\@author}
                \fi
            }};},
    fullColorBkgnd/.pic={
        \ifstandard
            \fill[colorB] (-6,-4.5) rectangle (6,4.5);
            \fill[colorC]
                (-6,3.0) -- (-5.25,3.75) -- (-0.75,-0.75) --
                (-3.75,-3.75) -- (-6,-1.5)
                (6,-3.0) -- (5.25,-3.75) -- (0.75,0.75) --
                (3.75,3.75) -- (6,1.5);
            \fill[colorA]
                (-6,4.5) -- (-4.5,4.5) -- (-5.25,3.75)
                (-3,-4.5) -- (-4.5,-4.5) -- (-3.75,-3.75)
                (6,-4.5) -- (4.5,-4.5) -- (5.25,-3.75)
                (3,4.5) -- (4.5,4.5) -- (3.75,3.75);
            \fill[colorD]
                (-4.5,3.75) -- (-3.75,4.5) -- (2.25,4.5) --
                (3,3.75) -- (-0.75,0)
                (4.5,-3.75) -- (3.75,-4.5) -- (-2.25,-4.5) --
                (-3,-3.75) -- (0.75,0)
                (-4.5,-3.75) -- (-5.25,-4.5) -- (-6,-4.5) -- (-6,-2.25)
                (4.5,3.75) --(5.25,4.5) -- (6,4.5) -- (6,2.25);
        \else
            \fill[colorB] (-8,-4.5) rectangle (8,4.5);
            \fill[colorC]
                (-8,3.0) -- (-7.25,3.75) -- (-2.75,-0.75) --
                (-5.75,-3.75) -- (-8,-1.5)
                (8,-3.0) -- (7.25,-3.75) -- (2.75,0.75) --
                (5.75,3.75) -- (8,1.5);
            \fill[colorA]
                (-8,4.5) -- (-6.5,4.5) -- (-7.25,3.75)
                (-5,-4.5) -- (-6.5,-4.5) -- (-5.75,-3.75)
                (8,-4.5) -- (6.5,-4.5) -- (7.25,-3.75)
                (5,4.5) -- (6.5,4.5) -- (5.75,3.75)
                (-2.75,-0.75) -- (0.75,2.75) --
                (2.75,0.75) -- (-0.75,-2.75);
            \fill[colorD]
                (-6.5,3.75) -- (-5.75,4.5) -- (1.75,4.5) -- (-2.75,0)
                (-6.5,-3.75) -- (-7.25,-4.5) -- (-8,-4.5) -- (-8,-2.25)
                (6.5,-3.75) -- (5.75,-4.5) -- (-1.75,-4.5) -- (2.75,0)
                (6.5,3.75) --(7.25,4.5) -- (8,4.5) -- (8,2.25);
        \fi},
    chapterpage/.pic={
        \path (0,0) pic{fullColorBkgnd};
        \node[inner sep=0, text width={1.53\@xw},
                white, align=flush center]
            at (0,0) {\huge\bfseries\strut\@headername\strut};},
    sectionpage/.pic={
        \fill[colorB] (-\@xw,-4.5) rectangle ({-\@xw+1cm},4.5);
        \fill[colorC] (-\@xw,3) -- ({-\@xw+0.75cm},3.75) --
            ({-\@xw+1cm},3.5) -- ({-\@xw+1cm},-2.5) -- (-\@xw,-1.5);
        \fill[colorA] (-\@xw,4.5) -- ({-\@xw+1cm},4.5) --
            ({-\@xw+1cm},4) -- ({-\@xw+0.75cm},3.75);
        \fill[colorD] (-\@xw,-4.5) -- (-\@xw,-2.25) --
            ({-\@xw+1cm},-3.25) -- ({-\@xw+1cm},-4.25) --
            ({-\@xw+0.75cm},-4.5);
        \node[right, inner sep=0, text depth=0]
            at ({-\@xw+1.5cm},3.723)
            {\begin{adjustbox}{max width={2\@xw-2.5cm}}
                    \Large\bfseries\ignorespaces\@headername\strut
                \end{adjustbox}};
        \phantomsection\addcontentsline{toc}{section}{\@altheadername}
        \node[inner sep=0, text depth=0, colorH]
            at ({-\@xw+0.5cm},-4) {\bfseries\thepage\strut};},
    closingpage/.pic={
        \path (0,0) pic{fullColorBkgnd};
        \path (0,0) pic[scale=2, colorH] {antlogocompass};}}

% Headers and footers
\newcommand{\@headername}{}
\pagestyle{fancy}
\fancyhead[L,R]{}
\fancyhead[C]{\tikz[overlay]{\path (\@xHead,\@yHead) pic{sectionpage};}}
\fancyfoot[L,C,R]{}
\renewcommand{\headrulewidth}{0pt}
\renewcommand{\footrulewidth}{0pt}

% Title page
\renewcommand{\maketitle}{
    \clearpage\thispagestyle{empty}\justifying\onecolumn%
    \tikz[overlay, xshift=\@xShift, yshift=\@yShift]{
        \path (0,0) pic{titlepage};}
    \clearpage}

% Chapter page
\newcommand{\chapter}[2][]{%
    \clearpage\onecolumn\justifying\thispagestyle{empty}\normalsize%
    \renewcommand{\@headername}{#2}%
    \ifx&#1&
        \def\@altheadername{#2}
    \else
        \def\@altheadername{#1}
    \fi
    \phantomsection\addcontentsline{toc}{chapter}{\@altheadername}
    \tikz[overlay, xshift=\@xShift, yshift=\@yShift]{
        \path (0,0) pic{chapterpage};}%
    \clearpage}

% \l@chapter: article.cls no lo define (solo lo hacen book/report), pero
% \chapter aquí sí llama \addcontentsline{toc}{chapter}{...}, así que hace
% falta para que \tableofcontents pueda tipografiar esas entradas.
\newcommand*\l@chapter[2]{%
  \ifnum \c@tocdepth >\m@ne
    \addpenalty{-\@highpenalty}%
    \vskip 1.0em \@plus\p@
    \setlength\@tempdima{1.5em}%
    \begingroup
      \parindent \z@ \rightskip \@pnumwidth
      \parfillskip -\@pnumwidth
      \leavevmode \bfseries
      \advance\leftskip\@tempdima
      \hskip -\leftskip
      #1\nobreak\hfil
      \nobreak\hb@xt@\@pnumwidth{\hss #2}\par
      \penalty\@highpenalty
    \endgroup
  \fi}

% Header name and sections
\renewcommand{\section}[2][]{%
    \clearpage\justifying\onecolumn\normalsize%
    \renewcommand{\@headername}{#2}%
    \ifx&#1&
        \def\@altheadername{#2}
    \else
        \def\@altheadername{#1}
    \fi}

% Other sections
\renewcommand{\subsection}[1]{\par\textcolor{colorH}{\textbf{#1}}\par}
\renewcommand{\subsubsection}[1]{\par\textbf{#1}\par}
\renewcommand{\paragraph}[1]{\par\textit{#1}\par}
\renewcommand{\subparagraph}[1]{\par\textit{#1}\par}

% Blank page
\newcommand\blankpage{\clearpage\null\thispagestyle{empty}}

% Closing page
\newcommand{\closing}{
    \clearpage\thispagestyle{empty}\justifying\onecolumn\normalsize%
    \tikz[overlay, xshift=\@xShift, yshift=\@yShift]{
        \path (0,0) pic{closingpage};}%
    \clearpage}