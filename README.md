# [flomonster.fr](http://flomonster.fr/)

## Build

Pages are written using reStructuredText, because it is way nicer than raw HTML.
A custom Python script converts RST to HTML (using [pandoc](https://pandoc.org/)
and [jinja](http://jinja.pocoo.org/) templating), which gives me great control
over how I write content. For every page there is a `.meta` file providing basic
information for the page, where to store it, what to template to use, etc...

```bash
pipenv install
pipenv shell
make build
```

Note: despite the Python virtual env, you still need to install a recent version
of pandoc because pypandoc is only a wrapper around it.

## Docker Image

You can find the docker image [here](https://hub.docker.com/r/flomonster/flomonster.fr).

## License

The content of this website is licensed under the
[Creative Commons license](http://creativecommons.org/licenses/by-nc-sa/4.0/),
and the source code in this project is licensed under the
[MIT license](http://opensource.org/licenses/mit-license.php).

## Credit

The sources for this website came mainly from https://github.com/haltode/haltode.fr.
