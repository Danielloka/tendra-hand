import bomData from "@content/data/bom.json";
import { Tag } from "@/components/ui/Tag";
import { formatDate } from "./format";
import "./pages.css";

type BomItem = { part: string; qty: number | string; spec: string; notes: string; link?: string; status: "have" | "planned" };
type Bom = { updated: string; groups: { name: string; items: BomItem[] }[] };

const bom = bomData as Bom;

/** Text that is (or ends in) "TODO: …" shows the TODO part muted, so gaps stand out as gaps. */
function WithTodo({ text }: { text: string | number }) {
  const s = String(text);
  const i = s.indexOf("TODO");
  return (
    <span>
      {i === -1 ? s : s.slice(0, i)}
      {i !== -1 && <span className="bom__todo">{s.slice(i)}</span>}
    </span>
  );
}

/**
 * Bill of materials from content/data/bom.json: one table per group. Below
 * 40rem each row becomes a stacked card (labels come from data-label).
 */
export function BomTable() {
  return (
    <div className="bom not-prose">
      {bom.groups.map((group) => (
        <div key={group.name} className="bom__group">
          <table className="bom__table">
            <caption className="bom__caption">{group.name}</caption>
            <thead>
              <tr>
                <th scope="col">Part</th>
                <th scope="col">Qty</th>
                <th scope="col">Spec</th>
                <th scope="col">Notes</th>
                <th scope="col">Status</th>
              </tr>
            </thead>
            <tbody>
              {group.items.map((item) => (
                <tr key={item.part}>
                  <th scope="row" className="bom__part">
                    {item.link ? <a href={item.link}>{item.part}</a> : item.part}
                  </th>
                  <td data-label="Qty" className="bom__qty">
                    <WithTodo text={item.qty} />
                  </td>
                  <td data-label="Spec">
                    <WithTodo text={item.spec} />
                  </td>
                  <td data-label="Notes" className="bom__notes">
                    <WithTodo text={item.notes} />
                  </td>
                  <td data-label="Status" className="bom__status">
                    {item.status === "have" ? (
                      <Tag tone="success" dot>
                        Have
                      </Tag>
                    ) : (
                      <Tag dot>Planned</Tag>
                    )}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      ))}
      <p className="caption bom__updated">
        Updated <time dateTime={bom.updated}>{formatDate(bom.updated)}</time>
      </p>
    </div>
  );
}
