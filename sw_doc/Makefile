doc : doc-html doc-pdf


doc-html :
	@asciidoctor -r asciidoctor-diagram filwndr.adoc 

doc-pdf :
	@asciidoctor-pdf -r asciidoctor-diagram filwndr.adoc

.PHONY : clean
clean :
	@rm -f filwndr.pdf filwndr.html

