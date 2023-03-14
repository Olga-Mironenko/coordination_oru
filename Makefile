all: \
	build/resources/main/index.html \
	build/resources/main/Visualization.js

build/resources/main/index.html: src/main/resources/index.html
	cp -a $< $@
	
build/resources/main/Visualization.js: src/main/resources/Visualization.js
	cp -a $< $@
