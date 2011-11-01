#!/usr/bin/env python
s=open('/etc/texmf/texmf.cnf','r').read()
if not s.startswith('CMAPFONTS'):
    open('/etc/texmf/texmf.cnf','w').write('CMAPFONTS = .;\$TEXMF/fonts/cmap//;/usr/share/fonts/cmap//\n'+s)
