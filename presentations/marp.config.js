// Marp engine that renders Mermaid via Kroki (kroki.io) as static SVG.
//
// No Docker and no extra npm packages: Node's built-in zlib produces the Kroki
// URL, and each ```mermaid fence becomes an <img> pointing at
// kroki.io/mermaid/svg/<encoded>. Diagrams render server-side, so box sizing is
// consistent and the output is export-safe (HTML / PDF / PPTX / PNG).
//
// Network: kroki.io is contacted when the deck is viewed or exported (the <img>
// loads then), and the diagram source is sent to the public server. To keep it
// fully local, self-host Kroki and set KROKI_BASE, e.g.
//   $env:KROKI_BASE = 'http://localhost:8000'
//
// Requires only: @marp-team/marp-core

const { Marp } = require('@marp-team/marp-core')
const zlib = require('zlib')

const KROKI_BASE = process.env.KROKI_BASE || 'https://kroki.io'

// Kroki GET encoding: zlib-deflate the source, then URL-safe base64.
function krokiSrc(type, source) {
  const compressed = zlib.deflateSync(Buffer.from(source, 'utf8'), { level: 9 })
  const encoded = compressed.toString('base64').replace(/\+/g, '-').replace(/\//g, '_')
  return `${KROKI_BASE}/${type}/svg/${encoded}`
}

// Prepend a theme directive so Kroki's Mermaid matches the deck styling.
function withTheme(source) {
  return /^\s*%%\{\s*init/.test(source)
    ? source
    : `%%{init: {'theme':'neutral'}}%%\n${source}`
}

const RUNTIME = `
<style>
  section p.kroki { margin: 0.2em auto; text-align: center; }
  section p.kroki img {
    max-width: 100%;
    max-height: 500px;
    height: auto;
  }
  /* Auto-fit wrapper: keep width bound to the slide, scale from the top. */
  section > div.marp-autofit { width: 100%; }
</style>
<script>
  function autofitSlides() {
    document.querySelectorAll('section').forEach(function (section) {
      var wrap = section.querySelector(':scope > div.marp-autofit')
      if (!wrap) {
        wrap = document.createElement('div')
        wrap.className = 'marp-autofit'
        Array.prototype.slice.call(section.childNodes).filter(function (n) {
          return !(n.nodeType === 1 && (n.tagName === 'HEADER' || n.tagName === 'FOOTER'))
        }).forEach(function (n) { wrap.appendChild(n) })
        section.appendChild(wrap)
      }
      wrap.style.transform = 'none'
      var cs = getComputedStyle(section)
      var availH = section.clientHeight - parseFloat(cs.paddingTop) - parseFloat(cs.paddingBottom)
      var availW = section.clientWidth - parseFloat(cs.paddingLeft) - parseFloat(cs.paddingRight)
      var scale = Math.min(1, availH / wrap.scrollHeight, availW / wrap.scrollWidth)
      if (isFinite(scale) && scale < 1) {
        wrap.style.transformOrigin = 'top center'
        wrap.style.transform = 'scale(' + scale + ')'
      }
    })
  }
  // Wait for the Kroki SVGs to load before measuring, then auto-fit.
  function whenReady() {
    var imgs = Array.prototype.slice.call(document.querySelectorAll('p.kroki img'))
    var pending = imgs.filter(function (im) { return !im.complete })
    if (pending.length === 0) { requestAnimationFrame(autofitSlides); return }
    var left = pending.length
    pending.forEach(function (im) {
      var done = function () { if (--left === 0) requestAnimationFrame(autofitSlides) }
      im.addEventListener('load', done)
      im.addEventListener('error', done)
    })
  }
  if (document.readyState === 'complete') whenReady()
  else window.addEventListener('load', whenReady)
</script>
`

module.exports = {
  html: true,
  engine: (opts) => {
    const marp = new Marp(opts)
    const md = marp.markdown
    const defaultFence =
      md.renderer.rules.fence ||
      ((tokens, idx, options, env, self) => self.renderToken(tokens, idx, options))

    // Rewrite ```mermaid fences into a Kroki-rendered <img>.
    md.renderer.rules.fence = (tokens, idx, options, env, self) => {
      const token = tokens[idx]
      if ((token.info || '').trim().toLowerCase() === 'mermaid') {
        const src = krokiSrc('mermaid', withTheme(token.content))
        return `<p class="kroki"><img src="${src}" alt="diagram" /></p>`
      }
      return defaultFence(tokens, idx, options, env, self)
    }

    // Inject the fit CSS + auto-fit runtime once per rendered deck.
    const render = marp.render.bind(marp)
    marp.render = (markdown, env) => {
      const result = render(markdown, env)
      result.html = `${result.html}${RUNTIME}`
      return result
    }

    return marp
  },
}
