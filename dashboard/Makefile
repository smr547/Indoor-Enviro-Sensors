APP_NAME := weather-station-dashboard
VERSION := 0.1.0
SIGNALK_HOME := /home/signalk/.signalk
PKG := $(APP_NAME)-$(VERSION).tgz

.PHONY: pack install uninstall deploy restart status verify clean

pack:
	npm pack

install: pack
	sudo systemctl stop signalk
	sudo rm -rf $(SIGNALK_HOME)/node_modules/$(APP_NAME)
	sudo npm install ./$(PKG) --prefix $(SIGNALK_HOME) --save
	sudo chown -R signalk:signalk $(SIGNALK_HOME)
	sudo systemctl start signalk

deploy: install verify

uninstall:
	sudo systemctl stop signalk
	sudo npm uninstall $(APP_NAME) --prefix $(SIGNALK_HOME) --save || true
	sudo rm -rf $(SIGNALK_HOME)/node_modules/$(APP_NAME)
	sudo chown -R signalk:signalk $(SIGNALK_HOME)
	sudo systemctl start signalk

restart:
	sudo systemctl restart signalk

status:
	systemctl status signalk --no-pager

verify:
	sudo npm list $(APP_NAME) --prefix $(SIGNALK_HOME)

clean:
	rm -f *.tgz
