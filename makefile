

doc.pdf : README.md
	pandoc -V papersize:a4 --number-sections -o doc.pdf README.md

clean :
	rm doc.pdf
	make -C src clean

