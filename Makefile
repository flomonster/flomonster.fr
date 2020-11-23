WEBSITE_DIR = html

.PHONY: all build collectstatic runserver clean

all: build run

build:
	mkdir -p $(WEBSITE_DIR)
	python3 build/build.py $(WEBSITE_DIR)
	cp -TR css $(WEBSITE_DIR)/css
	cp -TR img $(WEBSITE_DIR)/img
	cp -TR static $(WEBSITE_DIR)/upload

run:
	cd $(WEBSITE_DIR) && python3 -m http.server 8000

clean:
	${RM} -r $(WEBSITE_DIR)
