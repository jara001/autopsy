# Self-documenting Makefile by prwhite
# https://gist.github.com/prwhite/8168133
# https://gist.github.com/prwhite/8168133#gistcomment-1716694
# https://gist.github.com/prwhite/8168133#gistcomment-1737630

.PHONY: build develop install uninstall reinstall

STASHABLE := $(shell git stash create)

help: ## Show this help message.
	@echo "Usage: make [target] ..."
	@echo
	@echo "Targets:"
	@grep --color=auto -F "## " $(MAKEFILE_LIST) | grep --color=auto -F -v grep | sed -e "s/\\$$//" | sed -e "s/##//" | column -c2 -t -s :
	@grep "##@[^ \"]*" $(MAKEFILE_LIST) | grep --color=auto -F -v grep | sed -e "s/^.*##@\\([a-zA-Z][a-zA-Z]*\\).*\$$/\1/" | sed "/^\\$$/d" | sort | uniq | xargs -I'{}' -n 1 bash -c "echo; echo {} targets:; grep '##@{}' $(MAKEFILE_LIST) | sed -e 's/##@{}//' | column -c2 -t -s :"

build: ##@Build Build all.
build: stash build-wheel unstash

build-wheel: ##@Build Build a universal Python wheel.
	python setup.py build bdist_wheel --universal

develop: ##@Developer Install the package as link to this repository.
	python setup.py develop --user

install: ##@Install Install the package for current user.
	python setup.py install --user

uninstall: ##@Install Uninstall the package.
	python -m pip uninstall autopsy

reinstall: ##@Install Reinstall the package
reinstall: uninstall install

stash:
	@# --include-untracked or even --all should be here, but:
	@#  a) I am using that for more stuff, so I have GBs of data here
	@#  b) version.py kinda breaks that, so I it needs to be deleted right after?
	@if test $(STASHABLE); then \
		git stash save --quiet "Prebuild stash"; \
	fi

unstash:
	@if test $(STASHABLE); then \
		git stash pop --quiet; \
	fi
