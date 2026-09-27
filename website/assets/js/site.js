/* ==========================================================================
   Tendra Hand — site behaviour. No dependencies.
   - theme toggle (light default, choice remembered)
   - glass nav on scroll
   - scroll reveals (blocks and line-by-line text)
   - count-up numbers
   - copy button on code blocks
   Everything respects prefers-reduced-motion.
   ========================================================================== */

(() => {
  const root = document.documentElement;
  const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)");

  /* ---- Theme ------------------------------------------------------------ */
  const THEME_KEY = "tendra-theme";

  function setTheme(theme) {
    if (theme === "dark") root.dataset.theme = "dark";
    else delete root.dataset.theme;
    try {
      localStorage.setItem(THEME_KEY, theme);
    } catch {
      /* storage blocked: the choice just won't be remembered */
    }
    document.querySelectorAll(".theme-toggle").forEach((btn) => {
      btn.setAttribute("aria-pressed", String(theme === "dark"));
    });
    document.dispatchEvent(new CustomEvent("themechange", { detail: { theme } }));
  }

  document.querySelectorAll(".theme-toggle").forEach((btn) => {
    btn.setAttribute("aria-pressed", String(root.dataset.theme === "dark"));
    btn.addEventListener("click", () => {
      setTheme(root.dataset.theme === "dark" ? "light" : "dark");
    });
  });

  /* ---- Nav: glass background once the page has scrolled ---------------- */
  const nav = document.querySelector(".nav");
  if (nav) {
    const sentinel = document.createElement("div");
    sentinel.style.cssText = "position:absolute;top:0;height:8px;width:1px;pointer-events:none";
    document.body.prepend(sentinel);
    new IntersectionObserver(([entry]) => {
      nav.classList.toggle("is-scrolled", !entry.isIntersecting);
    }).observe(sentinel);
  }

  /* ---- Line splitting for [data-reveal="lines"] -------------------------
     Wraps words, measures which visual line each lands on, then groups them
     into .reveal-line > .reveal-line__inner. After the animation, the
     original markup is restored so text reflows normally on resize. */
  function splitLines(el) {
    const original = el.innerHTML;
    const words = el.textContent.trim().split(/\s+/);
    el.setAttribute("aria-label", el.textContent.trim());
    el.innerHTML = words.map((w) => `<span class="reveal-word">${escapeHtml(w)}</span>`).join(" ");

    const lines = [];
    let lastTop = null;
    el.querySelectorAll(".reveal-word").forEach((span) => {
      const top = span.offsetTop;
      if (top !== lastTop) {
        lines.push([]);
        lastTop = top;
      }
      lines[lines.length - 1].push(span.textContent);
    });

    el.innerHTML = lines
      .map(
        (line, i) =>
          `<span class="reveal-line" aria-hidden="true"><span class="reveal-line__inner" style="--i:${i}">${escapeHtml(line.join(" "))}</span></span>`,
      )
      .join("");
    el.classList.add("is-split");

    const last = el.querySelector(`.reveal-line:last-child .reveal-line__inner`);
    last?.addEventListener(
      "transitionend",
      () => {
        el.innerHTML = original;
        el.removeAttribute("aria-label");
      },
      { once: true },
    );
  }

  function escapeHtml(s) {
    return s.replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c]);
  }

  /* ---- Count-up ---------------------------------------------------------- */
  function countUp(el) {
    const to = parseFloat(el.dataset.countTo);
    const decimals = parseInt(el.dataset.decimals || "0", 10);
    const duration = parseInt(el.dataset.duration || "1400", 10);
    const format = (v) =>
      v.toLocaleString("en-US", { minimumFractionDigits: decimals, maximumFractionDigits: decimals });

    if (reduceMotion.matches) {
      el.textContent = format(to);
      return;
    }
    const start = performance.now();
    const ease = (t) => 1 - Math.pow(1 - t, 4); // easeOutQuart, close to --ease-out
    const tick = (now) => {
      const t = Math.min((now - start) / duration, 1);
      el.textContent = format(to * ease(t));
      if (t < 1) requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  }

  /* ---- Observer: reveals + counters ------------------------------------- */
  const revealEls = document.querySelectorAll("[data-reveal]");
  const countEls = document.querySelectorAll("[data-count-to]");

  if (reduceMotion.matches || !("IntersectionObserver" in window)) {
    revealEls.forEach((el) => el.classList.add("is-visible"));
    countEls.forEach(countUp);
  } else {
    // Split once fonts are ready, otherwise line breaks are measured with fallback fonts.
    const fontsReady = document.fonts ? document.fonts.ready : Promise.resolve();
    fontsReady.then(() => {
      document.querySelectorAll('[data-reveal="lines"]').forEach(splitLines);

      const io = new IntersectionObserver(
        (entries) => {
          entries.forEach((entry) => {
            if (!entry.isIntersecting) return;
            const el = entry.target;
            if (el.hasAttribute("data-count-to")) countUp(el);
            else el.classList.add("is-visible");
            io.unobserve(el);
          });
        },
        { rootMargin: "0px 0px -10% 0px", threshold: 0.15 },
      );
      revealEls.forEach((el) => io.observe(el));
      countEls.forEach((el) => {
        el.textContent = "0";
        io.observe(el);
      });
    });
  }

  /* ---- Copy buttons on code blocks ------------------------------------- */
  document.querySelectorAll(".code__copy").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const pre = btn.closest(".code").querySelector("pre");
      try {
        await navigator.clipboard.writeText(pre.innerText);
        btn.dataset.copied = "";
        btn.textContent = "Copied";
      } catch {
        btn.textContent = "Select + Ctrl+C";
      }
      setTimeout(() => {
        delete btn.dataset.copied;
        btn.textContent = "Copy";
      }, 1600);
    });
  });
})();
