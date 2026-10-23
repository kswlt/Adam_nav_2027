// Foxglove ExtensionModule; no ROS control logic in this presentation layer.
export function activate(extensionContext) {
  extensionContext.registerPanel({
    name: 'Adam Z 操作台',
    initPanel(context) {
      const root = context.panelElement;
      root.style.cssText = 'height:100%;display:flex;flex-direction:column;background:#10171e;color:white';
      const form = document.createElement('form');
      form.style.cssText = 'display:flex;gap:6px;padding:6px;font:12px sans-serif';
      const input = document.createElement('input');
      input.placeholder = 'http://127.0.0.1:8878/#操作令牌';
      input.type = 'password'; input.style.cssText = 'flex:1;min-width:80px';
      // No token is ever written into panel state/shared layouts.
      input.value = 'http://127.0.0.1:8878/';
      const button = document.createElement('button'); button.textContent = '连接操作台';
      form.append(input, button);
      const frame = document.createElement('iframe');
      frame.title = 'ASUS Z建图导航与终端';
      frame.style.cssText = 'flex:1;width:100%;border:0;min-height:0';
      frame.src = 'http://127.0.0.1:8878/';
      form.onsubmit = event => {
        event.preventDefault();
        try {
          const url = new URL(input.value);
          if (url.hostname !== '127.0.0.1' || url.port !== '8878' || url.protocol !== 'http:') {
            throw Error('请使用本机SSH转发地址 http://127.0.0.1:8878/');
          }
          frame.src = url.toString(); input.value = 'http://127.0.0.1:8878/';
        } catch (error) { button.textContent = String(error.message); }
      };
      root.append(form, frame);
      return () => root.replaceChildren();
    },
  });
}
