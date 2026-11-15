// Coupled CARLA project page: theme button, video previews on hover, the player dialog, copy buttons.
(function () {
  "use strict";
  var root = document.documentElement;
  var mq = function (q) { return window.matchMedia && window.matchMedia(q).matches; };
  var reduced = mq("(prefers-reduced-motion: reduce)");
  var saveData = !!(navigator.connection && navigator.connection.saveData);
  var canHover = mq("(hover: hover) and (pointer: fine)");

  // light / dark: the button flips what is shown now and remembers it in this browser
  var themeBtn = document.querySelector(".theme");
  if (themeBtn) {
    themeBtn.addEventListener("click", function () {
      var dark = root.dataset.theme ? root.dataset.theme === "dark" : mq("(prefers-color-scheme: dark)");
      root.dataset.theme = dark ? "light" : "dark";
      try { localStorage.setItem("cc-theme", root.dataset.theme); } catch (e) {}
    });
  }

  function load(video) {
    if (!video.getAttribute("src") && video.dataset.src) video.src = video.dataset.src;
  }
  function preview(tile, on) {
    var v = tile.querySelector("video");
    if (!v) return;
    if (on) {
      load(v);
      var p = v.play();
      if (p && p.catch) p.catch(function () {});
      tile.classList.add("is-previewing");
    } else {
      v.pause();
      tile.classList.remove("is-previewing");
    }
  }

  var tiles = document.querySelectorAll(".clip-play, .hero-play");

  // muted preview while the pointer is on a clip (mouse or trackpad only, not when motion should be reduced)
  if (canHover && !reduced && !saveData) {
    tiles.forEach(function (tile) {
      tile.addEventListener("mouseenter", function () { preview(tile, true); });
      tile.addEventListener("mouseleave", function () { preview(tile, false); });
    });
  }

  // the hero clip plays muted while it is on screen (wide screens, no reduced motion, no data saver)
  var hero = document.querySelector(".hero-play");
  if (hero && !reduced && !saveData && mq("(min-width: 960px)") && "IntersectionObserver" in window) {
    new IntersectionObserver(function (entries) {
      entries.forEach(function (e) { preview(hero, e.isIntersecting); });
    }, { threshold: 0.4 }).observe(hero);
  }

  // the player: the clip with sound and controls
  var dialog = document.getElementById("player");
  var player = dialog && dialog.querySelector("video");
  var opener = null;
  function openPlayer(tile) {
    if (!dialog || typeof dialog.showModal !== "function") {
      window.location.href = tile.dataset.src;          // very old browsers: the file itself
      return;
    }
    opener = tile;
    tiles.forEach(function (t) { preview(t, false); });
    dialog.querySelector("#player-title").textContent = tile.dataset.title || "Video";
    player.poster = tile.dataset.poster || "";
    player.src = tile.dataset.src;
    player.muted = false;
    dialog.showModal();
    var p = player.play();
    if (p && p.catch) p.catch(function () {});
  }
  function closePlayer() {
    if (dialog.open) dialog.close();
  }
  if (dialog) {
    dialog.addEventListener("close", function () {
      player.pause();
      player.removeAttribute("src");
      player.load();                                    // stops the download
      if (opener) opener.focus();
    });
    dialog.querySelector(".player-close").addEventListener("click", closePlayer);
    dialog.addEventListener("click", function (e) { if (e.target === dialog) closePlayer(); });
  }
  tiles.forEach(function (tile) {
    tile.addEventListener("click", function () { openPlayer(tile); });
  });

  // copy buttons
  document.querySelectorAll("[data-copy]").forEach(function (btn) {
    btn.addEventListener("click", function () {
      var el = document.getElementById(btn.dataset.copy);
      if (!el || !navigator.clipboard) return;
      navigator.clipboard.writeText(el.textContent).then(function () {
        var was = btn.textContent;
        btn.textContent = "Copied";
        setTimeout(function () { btn.textContent = was; }, 1600);
      });
    });
  });
})();
