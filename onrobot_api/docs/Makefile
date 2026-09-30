# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SOURCEDIR     =
BUILDDIR      = build
OUTPUT_DIR    = $(BUILDDIR)/html

default: html

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

clean:
	rm -rf ./doxygen_cpp
	rm -rf ./build
	rm -rf ./xml

html: Makefile
	@doxygen
	@$(SPHINXBUILD) -M html "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
	@xargs -a strip.txt -I{} rm "${OUTPUT_DIR}/{}" 2> /dev/null || true

# Skip stripping the static files
html-full: Makefile
	@doxygen
	@$(SPHINXBUILD) -M html "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

