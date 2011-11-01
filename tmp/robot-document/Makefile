all:
	for rstfile in `ls *.rst` ; do\
	  echo "convert from $$rstfile to `basename $$rstfile .rst`.pdf"; \
	  rst2pdf -s rst2pdf_default.style -o `basename $$rstfile .rst`.pdf $$rstfile  --font-path=migu-2m-20111002/;\
	done

clean:
	-rm *~ *.pdf

