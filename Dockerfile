FROM --platform=${BUILDPLATFORM} python:3.10-alpine AS builder
ARG TARGETPLATFORM
ARG BUILDPLATFORM

COPY . /code

RUN apk add --no-cache \
    --repository http://dl-cdn.alpinelinux.org/alpine/edge/testing/ \
    make \
    pandoc

WORKDIR /code

RUN pip install --no-cache-dir -r requirements.txt

RUN make build

# Run stage
FROM nginx:alpine

ARG NGINX_CONFIG=nginx.conf
COPY --from=builder /code/html /usr/share/nginx/html

RUN rm /etc/nginx/conf.d/default.conf
COPY nginx/$NGINX_CONFIG /etc/nginx/conf.d

EXPOSE 80

ENTRYPOINT nginx -g "daemon off;"
