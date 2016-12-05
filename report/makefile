report:
	pdflatex -output-directory=build masdr
	cp references.bib build
	cd build && bibtex masdr
	pdflatex -output-directory=build masdr
	pdflatex -output-directory=build masdr
	mv build/masdr.pdf .

rebuild: clean report

clean:
	rm -f masdr.pdf
	rm -rf build
	mkdir build
