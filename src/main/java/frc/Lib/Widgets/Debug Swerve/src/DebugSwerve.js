class DebugSwerve extends HTMLElement {
    constructor() {
      super();
      this.attachShadow({ mode: "open" });
  
      this.shadowRoot.innerHTML = `
        <style>
          :host {
            display: block;
            width: 100%;
            height: 100%;
            background: #1e1e1e;
          }
          canvas {
            width: 100%;
            height: 100%;
          }
        </style>
        <canvas></canvas>
      `;
  
      this.canvas = this.shadowRoot.querySelector("canvas");
      this.ctx = this.canvas.getContext("2d");
  
      this.actual = [];
      this.desired = [];
    }
  
    connectedCallback() {
      this.resizeCanvas();
      window.addEventListener("resize", () => this.resizeCanvas());
  
      this.nt4Client = this.querySelector("nt4-client");
  
      const actualTopic = this.getAttribute("actual-topic");
      const desiredTopic = this.getAttribute("desired-topic");
  
      if (this.nt4Client) {
        if (actualTopic) {
          this.nt4Client.subscribe(actualTopic, (_, value) => {
            this.actual = value || [];
            this.draw();
          });
        }
  
        if (desiredTopic) {
          this.nt4Client.subscribe(desiredTopic, (_, value) => {
            this.desired = value || [];
            this.draw();
          });
        }
      }
    }
  
    resizeCanvas() {
      this.canvas.width = this.clientWidth;
      this.canvas.height = this.clientHeight;
      this.draw();
    }
  
    draw() {
      const ctx = this.ctx;
      const w = this.canvas.width;
      const h = this.canvas.height;
  
      ctx.clearRect(0, 0, w, h);
  
      const transform = (pt) => ({
        x: w / 2 + pt.x * 50,
        y: h / 2 - pt.y * 50
      });
  
      const drawVector = (x, y, angleDeg, color) => {
        const len = 30;
        const rad = angleDeg * Math.PI / 180;
        const dx = Math.cos(rad) * len;
        const dy = Math.sin(rad) * len;
  
        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(x + dx, y - dy);
        ctx.stroke();
  
        // Draw small circle at base
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(x, y, 3, 0, Math.PI * 2);
        ctx.fill();
      };
  
      for (let mod of this.actual) {
        const { x, y, angleDeg } = mod;
        const pt = transform({ x, y });
        drawVector(pt.x, pt.y, angleDeg, "lime");
      }
  
      for (let mod of this.desired) {
        const { x, y, angleDeg } = mod;
        const pt = transform({ x, y });
        drawVector(pt.x, pt.y, angleDeg, "cyan");
      }
    }
  }
  
  customElements.define("Debug Swerve", DebugSwerve);
  