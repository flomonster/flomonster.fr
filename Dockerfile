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
FROM python:3.8-alpine

COPY --from=builder /code/html /site

WORKDIR /site

CMD [ "python3", "-m", "http.server", "8000" ]
