AUTOMAKE_OPTIONS = no-dependencies
ACLOCAL_AMFLAGS = -I support
CLEANFILES = config-stamp

SUBDIRS = support tools src share doc
DIST_SUBDIRS = $(SUBDIRS) examples

# Force the installation before running the testsuite
examples:: install
	$(MAKE) -C examples build-all

check-local: examples #run-examples

run-examples:: examples
	$(MAKE) -C examples check-all

clean-examples::
	$(MAKE) -C examples clean

distclean-examples::
	$(MAKE) -C examples distclean

po_hi_doc_dir=${shell $(CYGPATH_U) '$(OCARINA_PREFIX)/share/doc/ocarina'}

install-data-local:
	$(MAKE) -C examples DESTDIR="$(DESTDIR)" install
	$(INSTALL) -d $(DESTDIR)$(po_hi_doc_dir)

uninstall-local:
	$(MAKE) -C examples DESTDIR="$(DESTDIR)" uninstall

debug-makefile:
	@echo "GNATMAKE           = $(GNATMAKE)"
	@echo "GNATMAKE_FOR_LEON  = $(GNATMAKE_FOR_LEON)"
	@echo "GNATMAKE_FOR_ERC32 = $(GNATMAKE_FOR_ERC32)"
	$(MAKE) -C examples debug-makefile

release:
	rm -rf release/
	mkdir release/
	$(MAKE) dist
	cp -f $(DIST_ARCHIVES) release/$(DIST_ARCHIVES)

# We disable checking of the distclean rule, it is a nuisance
distcleancheck:
	@: